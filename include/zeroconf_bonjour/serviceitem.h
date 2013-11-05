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

#include <string>

#include "dns_sd.h"

class ServiceItem {
public:
	ServiceItem() {
		rootref = NULL;
		subref = NULL;
		hostport = 0;
	}
	virtual ~ServiceItem() {
	}
public:
	DNSServiceRef rootref;
	DNSServiceRef subref;
	std::basic_string<char> type;
	std::basic_string<char> name;
	std::basic_string<char> hostname;
	std::basic_string<char> hostip;
	unsigned int hostport;
	std::basic_string<char> domain;
	std::basic_string<char> protocol;
	std::basic_string<char> description;
};
