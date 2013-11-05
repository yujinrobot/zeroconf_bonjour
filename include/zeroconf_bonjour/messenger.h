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

#pragma once

#include <ros/ros.h>

#include <zeroconf_msgs/DiscoveredService.h>
#include <zeroconf_msgs/PublishedService.h>

#include <zeroconf_msgs/AddListener.h>
#include <zeroconf_msgs/RemoveListener.h>
#include <zeroconf_msgs/AddService.h>
#include <zeroconf_msgs/RemoveService.h>
#include <zeroconf_msgs/ListDiscoveredServices.h>
#include <zeroconf_msgs/ListPublishedServices.h>

class roshandler 
{
public:
	roshandler();
	virtual ~roshandler();
private:
	ros::NodeHandle* _Node;
	HANDLE _Spin;
public:
	ros::NodeHandle* GetNode();
	void begin(const char* name);
	void end();
private:
	static unsigned long spin(void* arg);
};

class Bonjour;

class Messenger
{
public:
	Messenger(HWND hWnd, Bonjour* pBonjour);
	virtual ~Messenger();

private:
	HWND m_hWnd;
	Bonjour* m_pBonjour;

private:
	roshandler* m_roshandler;
	ros::Publisher* m_pubDiscovered;
	ros::Publisher* m_pubPublished;
	ros::ServiceServer* m_srvAddListener;
	ros::ServiceServer* m_srvRemoveListener;
	ros::ServiceServer* m_srvAddService;
	ros::ServiceServer* m_srvRemoveService;
	ros::ServiceServer* m_srvListDiscovered;
	ros::ServiceServer* m_srvListPublished;

public:
	BOOL Initialize();
	void Uninitialize();

public:
	bool AddListener(zeroconf_msgs::AddListener::Request& req, 
		zeroconf_msgs::AddListener::Response& res);
	bool RemoveListener(zeroconf_msgs::RemoveListener::Request& req, 
		zeroconf_msgs::RemoveListener::Response& res);
	bool AddService(zeroconf_msgs::AddService::Request& req, 
		zeroconf_msgs::AddService::Response& res);
	bool RemoveService(zeroconf_msgs::RemoveService::Request& req, 
		zeroconf_msgs::RemoveService::Response& res);
	bool ListDiscoveredServices(zeroconf_msgs::ListDiscoveredServices::Request& req, 
		zeroconf_msgs::ListDiscoveredServices::Response& res);
	bool ListPublishedServices(zeroconf_msgs::ListPublishedServices::Request& req, 
		zeroconf_msgs::ListPublishedServices::Response& res);

};

