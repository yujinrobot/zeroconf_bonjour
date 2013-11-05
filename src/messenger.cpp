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

#include "../include/zeroconf_bonjour/stdafx.h"
#include "../include/zeroconf_bonjour/messenger.h"
#include "../include/zeroconf_bonjour/basetype.h"
#include "../include/zeroconf_bonjour/bonjour.h"

#include <tchar.h>

roshandler::roshandler() 
{
	_Node = NULL;
	_Spin = NULL;
}

roshandler::~roshandler()
{
	end();
}

ros::NodeHandle* roshandler::GetNode() 
{
	return _Node;
}

void roshandler::begin(const char* name) 
{
	int num = 0;
	char* arg = NULL;
	ros::init(num, &arg, name);
	_Node = new ros::NodeHandle;
	unsigned long threadid;
	_Spin = ::CreateThread(NULL,
		0,
		(LPTHREAD_START_ROUTINE)spin,
		(LPVOID)this,
		0,
		&threadid);
}

void roshandler::end() 
{
	ros::shutdown();
	if (_Node) {
		delete _Node;
		_Node = NULL;
	}
	if (_Spin) {
		if (::WaitForSingleObject(_Spin, 3000) == WAIT_TIMEOUT) {
			unsigned long exitcode;
			if (::GetExitCodeThread(_Spin, &exitcode)) {
				if (exitcode == 0x0103) {
					::TerminateThread(_Spin, exitcode);
				}
			}
		}
		::CloseHandle(_Spin);
		_Spin = NULL;
	}
}

unsigned long roshandler::spin(void* arg) 
{
	ros::spin();
	return 0;
}

Messenger::Messenger(HWND hWnd, Bonjour* pBonjour)
{
	m_hWnd = hWnd;
	m_pBonjour = pBonjour;
	m_roshandler = new roshandler;
	m_pubDiscovered = NULL;
	m_pubPublished = NULL;
	m_srvRemoveListener = NULL;
	m_srvAddService = NULL;
	m_srvRemoveService = NULL;
	m_srvListDiscovered = NULL;
	m_srvListPublished = NULL;

}

Messenger::~Messenger()
{
	Uninitialize();

	delete m_roshandler;
	m_roshandler = NULL;

}

BOOL Messenger::Initialize()
{
	m_roshandler->begin("zeroconf");
	ros::NodeHandle* node = m_roshandler->GetNode();

	//m_pubDiscovered = new ros::Publisher(node->advertise<zeroconf_msgs::DiscoveredService>("DiscoveredService", 100));
	//m_pubPublished = new ros::Publisher(node->advertise<zeroconf_msgs::PublishedService>("PublishedService", 100));

	m_srvAddListener = new ros::ServiceServer(node->advertiseService<Messenger, zeroconf_msgs::AddListener::Request, 
		zeroconf_msgs::AddListener::Response>("add_listener", &Messenger::AddListener, this));
	m_srvRemoveListener = new ros::ServiceServer(node->advertiseService<Messenger, zeroconf_msgs::RemoveListener::Request, 
		zeroconf_msgs::RemoveListener::Response>("remove_listener", &Messenger::RemoveListener, this));
	m_srvAddService = new ros::ServiceServer(node->advertiseService<Messenger, zeroconf_msgs::AddService::Request, 
		zeroconf_msgs::AddService::Response>("add_service", &Messenger::AddService, this));
	m_srvRemoveService = new ros::ServiceServer(node->advertiseService<Messenger, zeroconf_msgs::RemoveService::Request, 
		zeroconf_msgs::RemoveService::Response>("remove_service", &Messenger::RemoveService, this));
	m_srvListDiscovered = new ros::ServiceServer(node->advertiseService<Messenger, zeroconf_msgs::ListDiscoveredServices::Request, 
		zeroconf_msgs::ListDiscoveredServices::Response>("list_discovered_services", &Messenger::ListDiscoveredServices, this));
	m_srvListPublished = new ros::ServiceServer(node->advertiseService<Messenger, zeroconf_msgs::ListPublishedServices::Request, 
		zeroconf_msgs::ListPublishedServices::Response>("list_published_services", &Messenger::ListPublishedServices, this));

	return TRUE;
}

void Messenger::Uninitialize()
{
	if (m_pubDiscovered)
	{
		delete m_pubDiscovered;
		m_pubDiscovered = NULL;
	}

	if (m_pubPublished)
	{
		delete m_pubPublished;
		m_pubPublished = NULL;
	}

	if (m_srvAddListener)
	{
		delete m_srvAddListener;
		m_srvAddListener = NULL;
	}

	if (m_srvRemoveListener)
	{
		delete m_srvRemoveListener;
		m_srvRemoveListener = NULL;
	}

	if (m_srvAddService)
	{
		delete m_srvAddService;
		m_srvAddService = NULL;
	}

	if (m_srvRemoveService)
	{
		delete m_srvRemoveService;
		m_srvRemoveService = NULL;
	}

	if (m_srvListDiscovered)
	{
		delete m_srvListDiscovered;
		m_srvListDiscovered = NULL;
	}

	if (m_srvListPublished)
	{
		delete m_srvListPublished;
		m_srvListPublished = NULL;
	}

	m_roshandler->end();
}

bool Messenger::AddListener(zeroconf_msgs::AddListener::Request& req, 
	zeroconf_msgs::AddListener::Response& res)
{
	std::cout << "[zeroconf] AddListener(" << req.service_type << ")" << std::endl;

	::SendMessage(m_hWnd, WM_DISCOVER_ADD_SERVICE, (WPARAM)req.service_type.data(), 0);

	res.result = true;

	return true;
}

bool Messenger::RemoveListener(zeroconf_msgs::RemoveListener::Request& req, 
	zeroconf_msgs::RemoveListener::Response& res)
{
	std::cout << "[zeroconf] RemoveListener(" << req.service_type << ")" << std::endl;

	::SendMessage(m_hWnd, WM_DISCOVER_REMOVE_SERVICE, (WPARAM)req.service_type.data(), 0);

	res.result = true;

	return true;
}

bool Messenger::AddService(zeroconf_msgs::AddService::Request& req, 
	zeroconf_msgs::AddService::Response& res)
{
	std::cout << "[zeroconf] AddService(" << req.service.name << ", " << req.service.type << ", " << req.service.port << ")" << std::endl;

	ServiceItem si;
	si.name = req.service.name;
	si.type = req.service.type;
	si.domain = req.service.domain;
	si.hostport = req.service.port;
	si.description = req.service.description;

	::SendMessage(m_hWnd, WM_PUBLISH_ADD_SERVICE, (WPARAM)&si, 0);

	res.result = true;

	return true;
}

bool Messenger::RemoveService(zeroconf_msgs::RemoveService::Request& req, 
	zeroconf_msgs::RemoveService::Response& res)
{
	std::cout << "[zeroconf] RemoveService(" << req.service.name << ", " << req.service.type << ")" << std::endl;

	ServiceItem si;
	si.name = req.service.name;
	si.type = req.service.type;

	::SendMessage(m_hWnd, WM_PUBLISH_REMOVE_SERVICE, (WPARAM)&si, 0);

	res.result = true;

	return true;
}

bool Messenger::ListDiscoveredServices(zeroconf_msgs::ListDiscoveredServices::Request& req, 
	zeroconf_msgs::ListDiscoveredServices::Response& res)
{
	std::cout << "[zeroconf] ListDiscoveredServices" << std::endl;

	ServiceList scanned = m_pBonjour->GetServicesCached();

	for (ServiceList::iterator it=scanned.begin(); it!=scanned.end(); ++it)
	{
		if (!_tcscmp(req.service_type.data(), (*it)->type.data()))
		{
			zeroconf_msgs::DiscoveredService service;

			service.name = (*it)->name;
			service.type = (*it)->type;
			service.domain = (*it)->domain;
			service.description = (*it)->description;
			service.hostname = (*it)->hostname;
			service.ipv4_addresses.push_back((*it)->hostip);
			service.port = (*it)->hostport;

			res.services.push_back(service);

			std::cout << "[zeroconf] " << (*it)->name << ", " << (*it)->type << ", " << (*it)->hostip << ", " << (*it)->hostport << std::endl;
		}
	}

	return true;
}

bool Messenger::ListPublishedServices(zeroconf_msgs::ListPublishedServices::Request& req, 
	zeroconf_msgs::ListPublishedServices::Response& res)
{
	std::cout << "[zeroconf] ListPublishedServices" << std::endl;

	ServiceList published = m_pBonjour->GetPublished();

	for (ServiceList::iterator it=published.begin(); it!=published.end(); ++it)
	{
		if (!_tcscmp(req.service_type.data(), (*it)->type.data()))
		{
			zeroconf_msgs::PublishedService service;

			service.name = (*it)->name;
			service.type = (*it)->type;
			service.domain = (*it)->domain;
			service.port = (*it)->hostport;
			service.description = (*it)->description;

			res.services.push_back(service);

			std::cout << "[zeroconf] " << (*it)->name << ", " << (*it)->type << ", " << (*it)->hostport << std::endl;
		}
	}

	return true;
}

