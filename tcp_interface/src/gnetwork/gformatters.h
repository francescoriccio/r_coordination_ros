#ifndef _GFORMATTERS_H__
#define _GFORMATTERS_H__

#include <sys/types.h>
#include <netinet/in.h>
#include <iostream>

namespace GNetwork{

template <class X>
char* toNetworkByteArray(const X& d, char* buf){
	uint32_t s=sizeof(X);
	char* b=buf;
	switch (s){
	case 1: 
		*(static_cast<X*>((void*)b))=d;
		break;
	case 2 : 
		*(static_cast<int16_t*>((void*)b))=htons((int16_t)d);
		break;
	case 4 : 
		*(static_cast<int32_t*>((void*)b))=htonl((int32_t)d);
		break;
	default:
		s=0;
	}
	return b+s;
}

template <class X>
const char* fromNetworkByteArray(X& d, const char* buf){
	uint32_t s=sizeof(X);
	const char* b=buf;
	switch (s){
	case 1 : 
		d=*(static_cast<X*>((void*)b));
		break;
	case 2 :
		d=static_cast<X>(ntohs(*static_cast<int16_t*>((void*)b)));
		break;
	case 4 : 
		d=static_cast<X>(ntohl(*static_cast<int32_t*>((void*)b)));
		break;
	default:
		s=0;
	}
	return b+s;
}

template <class X>
char* toNetworkByteArray(uint32_t size, const X* d, char* buf){
	char* b=buf;
	b=toNetworkByteArray(size, buf);
	uint32_t s=sizeof(X);
	switch (s){
	case 1 : 
		for (uint32_t i=0; i<size; i++){
			*(static_cast<X*>((void*)b))=*d++;
			b+=s;
		}
		break;
	case 2 : 
		for (uint32_t i=0; i<size; i++){
			*(static_cast<int16_t*>((void*)b))=htons((int16_t)*d++);
			b+=s;
		}
		break;
	case 4 : 
		for (uint32_t i=0; i<size; i++){
			*(static_cast<int32_t*>((void*)b))=htonl((int32_t)*d++);
			b+=s;
		}
		break;
	default:
		s=0;
	}
	return b+s;
}

template <class X>
const char* fromNetworkByteArray(uint32_t& size, X* d, const char* buf){
	const char* b;
	b=fromNetworkByteArray(size, buf);
	uint32_t s=sizeof(X);
	switch (s){
	case 1 : 
		for (uint32_t i=0; i<size; i++){
			*d++=(static_cast<X>(*b));
			b+=s;
		}
		break;
	case 2 : 
		for (uint32_t i=0; i<size; i++){
			*d++=static_cast<X>(ntohs(*(int16_t*)b));
			b+=s;
		}
		break;
	case 4 : 
		for (uint32_t i=0; i<size; i++){
			*d++=static_cast<X>(ntohl(*(int32_t*)b));
			b+=s;
		}
		break;
	default:
		s=0;
	}
	return b+s;
}

};

#endif
