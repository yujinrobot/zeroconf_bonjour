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

#include "serviceitem.h"

#include <list>

class ServiceList : public std::list<ServiceItem*> {
public:
	ServiceList() {
	}
	ServiceList(const ServiceList& sl) {
		Empty();
		for (const_iterator ci=sl.begin(); ci!=sl.end(); ++ci)
			Add(**ci);
	}
	virtual ~ServiceList() {
		Empty();
	}
public:
	const ServiceList& operator =(const ServiceList& sl) {
		if (&sl == this) 
			return *this;
		Empty();
		for (const_iterator ci=sl.begin(); ci!=sl.end(); ++ci)
			Add(**ci);
		return *this;
	}
public:
	void Add(const ServiceItem& service) {
		ServiceItem* psi = new ServiceItem(service);
		push_back(psi);
	}
	void Remove(ServiceList::iterator si) {
		iterator it;
		for (it=begin(); it!=end(); ++it) {
			if (it == si)
				break;
		}
		if (it == end())
			return;
		delete *it;
		erase(it);
	}
	void Empty() {
		for (iterator it=begin(); it!=end(); ++it) {
			delete *it;
		}
		clear();
	}
};
