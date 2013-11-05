/*
 *
 * Copyright (c) 2013, Yujin Robot Co., Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms are permitted
 * provided that the above copyright notice and this paragraph are
 * duplicated in all such forms and that any documentation,
 * advertising materials, and other materials related to such
 * distribution and use acknowledge that the software was developed
 * by the Yujin Robot Co., Ltd.  The name of the
 * Yujin Robot Co., Ltd. may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED ''AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 *
 * Much of part in this source code originates from Mark Nelson
 * (http://marknelson.us/2011/10/25/dns-service-discovery-on-windows)
 *
 */

#include "../include/zeroconf_bonjour/stdafx.h"
#include "../include/zeroconf_bonjour/bonjour.h"

#include "../include/zeroconf_bonjour/basetype.h"

#include <tchar.h>
#include <stdio.h>
#include <string>
#include <Winsock2.h>
#include <Iprtrmib.h>
#include <Iphlpapi.h>
#include <crtdbg.h>
#include <iostream>
#include <iomanip>

Bonjour* Bonjour::m_pBonjour = NULL;

Bonjour::Bonjour(HWND hWnd)
{
	m_hTimerQueue = NULL;
	m_hTimer = NULL;
	m_pBonjour = this;
	m_hWnd = hWnd;
	m_bBusy = FALSE;

    HeapSetInformation(NULL, HeapEnableTerminationOnCorruption, NULL, 0);

	WSADATA	WSAData;
	WSAStartup(0x202, &WSAData);
}

Bonjour::~Bonjour()
{
	Uninitialize();

	m_pBonjour = NULL;
}


BOOL Bonjour::Initialize()
{
	m_hTimerQueue = ::CreateTimerQueue();

	return TRUE;
}

BOOL Bonjour::Uninitialize()
{
	End();

	if (m_hTimerQueue)
	{
		::DeleteTimerQueue(m_hTimerQueue);
		m_hTimerQueue = NULL;
	}

	return TRUE;
}

void Bonjour::Completed()
{
	End();
}

BOOL Bonjour::Begin()
{
	if (IsBusy())
	{
		// terminate browsing
		End();
	}

	for (std::unordered_map<DNSServiceRef, int>::iterator it=m_mapfd.begin(); it!=m_mapfd.end(); ++it)
		DNSServiceRefDeallocate(it->first);

	m_mapfd.clear();
	m_services.Empty();
	m_servicetypes.clear();

	DNSServiceRef client = NULL;
	DNSServiceErrorType err = DNSServiceBrowse(&client, 0, 0, "_services._dns-sd._udp", "", IterateServiceTypes, this);

	if (err != 0)
	{
		return FALSE;
	}

	SetBusy(TRUE);

	AddHandle(client);

	StartScanTimer();

	return TRUE;
}

void Bonjour::End()
{
	StopScanTimer();

	m_cached = m_services;
	SetBusy(FALSE);
}

BOOL Bonjour::StartScanTimer()
{
	if (!m_hTimerQueue)
		return FALSE;

	StopScanTimer();

	::CreateTimerQueueTimer(&m_hTimer, 
		m_hTimerQueue,
		(WAITORTIMERCALLBACK)ScanTimerRoutine,
		(LPVOID)this,
		250,
		250,
		0);
	
	return m_hTimer != NULL;
}

void Bonjour::StopScanTimer()
{
	if (!m_hTimerQueue)
		return;

	if (!m_hTimer)
		return;

	::DeleteTimerQueueTimer(m_hTimerQueue, m_hTimer, INVALID_HANDLE_VALUE);
	m_hTimer = NULL;
}

void Bonjour::ScanTimerRoutine(PVOID lpParam, BOOL bTimerOrWaitFired)
{
	if (!lpParam)
		return;

	Bonjour* ps = (Bonjour*)lpParam;

	::PostMessage(ps->GetWindow(), WM_SCAN_ENUMERATE, 0, 0);
}

void Bonjour::Scan()
{
	int count = 0;

	while (TRUE) 
	{
        if (m_mapfd.size() == 0) 
		{
			::PostMessage(GetWindow(), WM_SCAN_COMPLETE, 0, 0);
            break;
        }

		fd_set readfds;
		FD_ZERO(&readfds);

		for (std::unordered_map<DNSServiceRef, int>::const_iterator it=m_mapfd.cbegin(); it!=m_mapfd.cend(); it++) 
		{
			FD_SET(it->second, &readfds);
		}

		struct timeval tv = { 0, 1000 };
		
		if (select(0, &readfds, (fd_set*)NULL, (fd_set*)NULL, &tv) < 1)
			break;

        //
        // While iterating through the loop, the callback functions might delete
        // the client pointed to by the current iterator, so I have to increment
        // it BEFORE calling DNSServiceProcessResult
        //
		for (std::unordered_map<DNSServiceRef, int>::const_iterator it=m_mapfd.cbegin(); it!=m_mapfd.cend();) 
		{
            std::unordered_map<DNSServiceRef, int>::const_iterator prev = it++;

			if (FD_ISSET(prev->second, &readfds)) 
			{
				DNSServiceErrorType err = DNSServiceProcessResult(prev->first);
			}
		}

        if (++count > 10)
            break;
	}
}

void DNSSD_API Bonjour::IterateServiceTypes(DNSServiceRef sdref,
	DNSServiceFlags flags,
	uint32_t interfaceIndex,
	DNSServiceErrorType errorCode,
	const char *serviceName,
	const char *regtype,
	const char *replyDomain,
	void *context )
{
	Bonjour* pd = (Bonjour*)context;

	//
	// Service types are added to the top level of the tree
	//
	if (flags & kDNSServiceFlagsAdd && !errorCode) 
	{
		std::string r(regtype);
		size_t n = r.find_last_of('.');
		if (n != std::string::npos)
			r = r.substr(0, n);
		n = r.find_last_of('.');
		if (n != std::string::npos)
			r = r.substr(0, n);
		
		std::string service_type = serviceName;

		service_type += '.';
		service_type += r.c_str();
	
		std::unordered_set<std::string>::iterator it = pd->m_servicetypes.find(service_type);

        if (it == pd->m_servicetypes.end()) 
        {
			pd->AddType(service_type);

            DNSServiceRef client = NULL;
			DNSServiceErrorType err = DNSServiceBrowse(&client, 
				0, 
				0, 
				service_type.c_str(), 
				"", 
				IterateServiceInstances, 
				context);

			if (err == 0) 
			{
				pd->AddHandle(client);
            }
        }
	}

    if (!(flags & kDNSServiceFlagsMoreComing)) 
	{
		pd->RemoveHandle(sdref);
    }
}

void DNSSD_API Bonjour::IterateServiceInstances(DNSServiceRef sdref,
	DNSServiceFlags flags,
	uint32_t interfaceIndex,
	DNSServiceErrorType errorCode,
	const char *serviceName,
	const char *regtype,
	const char *replyDomain,
	void *context)
{
	Bonjour* pd = (Bonjour*)context;

	if ( (flags & kDNSServiceFlagsAdd) && !errorCode ) 
	{
		DNSServiceRef client = NULL;

		DNSServiceErrorType err = DNSServiceResolve(&client,
			0,
			interfaceIndex,
			serviceName,
			regtype,
			replyDomain,
			ResolveInstance,
			context );

		if (err == 0) 
		{
			pd->AddHandle(client);

			ServiceItem service;
			service.rootref = client;
			service.type = regtype;
			service.name = serviceName;
			service.domain = replyDomain;

			if (service.type.back() == _T('.'))
				service.type.pop_back();

			if (service.domain.back() == _T('.'))
				service.domain.pop_back();

			pd->AddService(service);
		} 
	}

	if (!(flags & kDNSServiceFlagsMoreComing)) 
	{
		pd->RemoveHandle(sdref);
	}
}

void DNSSD_API Bonjour::ResolveInstance(DNSServiceRef sdref,
	DNSServiceFlags flags,
	uint32_t interfaceIndex,
	DNSServiceErrorType errorCode,
	const char *fullname,
	const char *hosttarget,
	uint16_t port,
	uint16_t txtLen,
	const unsigned char *txtRecord,
	void *context)
{
	Bonjour* pd = (Bonjour*)context;

	if (!errorCode) 
	{
		ServiceList::iterator it;

		for (it=pd->m_services.begin(); it!=pd->m_services.end(); ++it)
		{
			if ((*it)->rootref == sdref)
				break;
		}

		if (it != pd->m_services.end())
		{
			DNSServiceRef client = NULL;
			char *adapterstr = _T("Unknown");
			
			MIB_IFROW mir;
			mir.dwIndex = interfaceIndex;
			DWORD result = GetIfEntry(&mir);
			
			if (result == 0) 
			{
				adapterstr = (char*)mir.bDescr;

				DNSServiceErrorType err = DNSServiceGetAddrInfo(&client,
					0x10000/*kDNSServiceFlagsTimeout*/,
					interfaceIndex,
					kDNSServiceProtocol_IPv4,
					hosttarget,
					GetAddress,
					context);

				if (err == 0) 
				{
					pd->AddHandle(client);
				} 
			}

			for (it=pd->m_services.begin(); it!=pd->m_services.end(); ++it)
			{
				if ((*it)->rootref == sdref)
					break;
			}

			if (it == pd->m_services.end())
			{
				_ASSERT(FALSE);
			}
			else
			{
				(*it)->subref = client;
				(*it)->hostname = hosttarget;
				(*it)->hostport = ntohs(port);

				size_t pos = (*it)->hostname.find(_T("."));

				if (pos != std::basic_string<char>::npos)
					(*it)->hostname.erase((*it)->hostname.begin() + pos, (*it)->hostname.end());
			}
		}
		else
		{
			_ASSERT(FALSE);
		}
	}

	if (!(flags & kDNSServiceFlagsMoreComing)) 
	{
		pd->RemoveHandle(sdref);
	}
}

void DNSSD_API Bonjour::GetAddress(DNSServiceRef sdref,
	DNSServiceFlags flags,
	uint32_t interfaceIndex,
	DNSServiceErrorType errorCode,
	const char *hostname,
	const struct sockaddr *address,
	uint32_t ttl,
	void *context)
{
	Bonjour* pd = (Bonjour*)context;

	if (!errorCode) 
	{
		ServiceList::iterator it;

		for (it=pd->m_services.begin(); it!=pd->m_services.end(); ++it)
		{
			if ((*it)->subref == sdref)
				break;
		}

		if (it != pd->m_services.end())
		{

			const sockaddr_in *in = (const sockaddr_in *) address;
			char *ip = inet_ntoa( in->sin_addr );

			for (it=pd->m_services.begin(); it!=pd->m_services.end(); ++it)
			{
				if ((*it)->subref == sdref)
					break;
			}

			if (it == pd->m_services.end())
			{
				_ASSERT(FALSE);
			}
			else
			{
				(*it)->hostip = ip;
				(*it)->protocol = "ipv4";
			}

		} 
		else
		{
			_ASSERT(FALSE);
		}
	}

	if (!(flags & kDNSServiceFlagsMoreComing)) 
	{
		pd->RemoveHandle(sdref);
	}
}

BOOL Bonjour::AddServiceToPublish(const char* service_name, 
	const char* service_type, 
	const char* domain, 
	unsigned int port,
	const char* description)
{
	DNSServiceRef client;
	unsigned short nbo_port = htons((unsigned short)port); // convert to network byte orger

	DNSServiceErrorType err = DNSServiceRegister(&client, 0, kDNSServiceInterfaceIndexAny, service_name, service_type, domain, 
		NULL, nbo_port, 0, NULL, PublishedService, (void*)this);

	ServiceItem service;
	service.rootref = client;
	service.type = service_type;
	service.name = service_name;
	service.domain = domain;
	service.hostport = port;

	m_published.Add(service);

	if (err != kDNSServiceErr_NoError)
	{
		return FALSE;
	}

	return TRUE;
}

BOOL Bonjour::RemoveServiceToPublish(const char* service_name, const char* service_type)
{
	ServiceList::iterator it;

	for (it=m_published.begin(); it!=m_published.end(); ++it)
	{
		if (!_tcscmp(service_name, (*it)->name.data()) && !_tcscmp(service_type, (*it)->type.data()))
			break;
	}

	if (it == m_published.end())
	{
		return FALSE;
	}

	DNSServiceRefDeallocate((*it)->rootref);
	m_published.Remove(it);

	return TRUE;
}

void DNSSD_API Bonjour::PublishedService(DNSServiceRef sdref,
	DNSServiceFlags flags,
	DNSServiceErrorType errorCode,
	const char *name,
	const char *regtype,
	const char *domain,
	void *context)
{
	Bonjour* pd = (Bonjour*)context;

	if (errorCode == kDNSServiceErr_NoError)
	{
		if (flags & kDNSServiceFlagsAdd) 
		{
			// published

			ServiceItem service;
			service.rootref = sdref;
			service.type = regtype;
			service.name = name;
			service.domain = domain;

			pd->m_published.Add(service);
		}
		else 
		{
			// removed
		}
	}
	else if (errorCode == kDNSServiceErr_NameConflict)
	{
		// duplicated
	}
	else
	{
		// failed to publish
	}

}

BOOL Bonjour::AddServiceToDiscover(const char* service_type)
{
	if (!service_type)
		return FALSE;

	m_listener.push_back(service_type);

	return TRUE;
}

BOOL Bonjour::RemoveServiceToDiscover(const char* service_type)
{
	if (!service_type)
		return FALSE;

	std::list<std::string>::iterator it;

	for (it=m_listener.begin(); it!=m_listener.end(); ++it)
	{
		if (!_tcscmp(service_type, it->data()))
			break;
	}

	if (it == m_listener.end())
		return FALSE;

	m_listener.erase(it);

	return TRUE;

}

void Bonjour::AddService(const ServiceItem& service)
{
	m_services.Add(service);
}

void Bonjour::AddType(const std::string& type)
{
	m_servicetypes.insert(type);
}

void Bonjour::AddHandle(DNSServiceRef ref)
{
	m_mapfd[ref] = DNSServiceRefSockFD(ref);
}

void Bonjour::RemoveHandle(DNSServiceRef ref)
{
	std::unordered_map<DNSServiceRef, int>::iterator it = m_mapfd.find(ref);

	if (it != m_mapfd.end())
	{
		DNSServiceRefDeallocate(it->first);
		m_mapfd.erase(it);
	}
}
