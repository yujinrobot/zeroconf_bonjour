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

#ifndef __DISCOVERY_H__
#define __DISCOVERY_H__

#include "basetype.h"
#include "servicelist.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>

class Bonjour
{
public:
	Bonjour(HWND hWnd);
	~Bonjour();

private:
	HWND m_hWnd;
	HANDLE m_hTimerQueue;
	HANDLE m_hTimer;
	static Bonjour* m_pBonjour;
	BOOL m_bBusy;

private:
	std::unordered_map<DNSServiceRef, int> m_mapfd;
	ServiceList m_services;
	ServiceList m_cached;
    std::unordered_set<std::string> m_servicetypes;

private:
	ServiceList m_published;
	std::list<std::string> m_listener;

private:
	BOOL StartScanTimer();
	void StopScanTimer();
	void AddService(const ServiceItem& service);
	void AddType(const std::string& type);
	void AddHandle(DNSServiceRef ref);
	void RemoveHandle(DNSServiceRef ref);
	static void CALLBACK ScanTimerRoutine(PVOID lpParam, BOOL bTimerOrWaitFired);

public:
	BOOL Initialize();
	BOOL Uninitialize();
	BOOL Begin();
	void End();
	HWND GetWindow() { return m_hWnd; }
	void SetBusy(BOOL bFlag) { m_bBusy = bFlag; }
	BOOL IsBusy() const { return m_bBusy; }
	void Completed();
	void Scan();
	const ServiceList& GetServices() const { return m_services; }
	const ServiceList& GetServicesCached() const { return m_cached; }
	const ServiceList& GetPublished() const { return m_published; }
	const std::list<std::string>& GetListener() const { return m_listener; }

public:
	BOOL AddServiceToPublish(const char* service_name, 
		const char* service_type, 
		const char* domain, 
		unsigned int port,
		const char* description);
	BOOL RemoveServiceToPublish(const char* service_name, const char* service_type);
	BOOL AddServiceToDiscover(const char* service_type);
	BOOL RemoveServiceToDiscover(const char* service_type);

private:
	static void DNSSD_API IterateServiceTypes(DNSServiceRef sdref,
		DNSServiceFlags flags,
		uint32_t interfaceIndex,
		DNSServiceErrorType errorCode,
		const char *serviceName,
		const char *regtype,
		const char *replyDomain,
		void *context);
	static void DNSSD_API IterateServiceInstances(DNSServiceRef sdref,
		DNSServiceFlags flags,
		uint32_t interfaceIndex,
		DNSServiceErrorType errorCode,
		const char *serviceName,
		const char *regtype,
		const char *replyDomain,
		void *context);
	static void DNSSD_API ResolveInstance(DNSServiceRef sdref,
		DNSServiceFlags flags,
		uint32_t interfaceIndex,
		DNSServiceErrorType errorCode,
		const char *fullname,
		const char *hosttarget,
		uint16_t port,
		uint16_t txtLen,
		const unsigned char *txtRecord,
		void *context);
	static void DNSSD_API GetAddress(DNSServiceRef sdref,
		DNSServiceFlags flags,
		uint32_t interfaceIndex,
		DNSServiceErrorType errorCode,
		const char *hostname,
		const struct sockaddr *address,
		uint32_t ttl,
		void *context);
	static void DNSSD_API PublishedService(DNSServiceRef sdref,
		DNSServiceFlags flags,
		DNSServiceErrorType errorCode,
		const char *name,
		const char *regtype,
		const char *domain,
		void *context);

};


#endif // __DISCOVERY_H__
