/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  The generate program idlc named byd_idl has been modified by BYD
  File name: app_currentpark.h
  Source: app_currentpark.idl
  Cyclone DDS: V0.9.1

*****************************************************************/
#ifndef DDSC_APP_CURRENTPARK_H
#define DDSC_APP_CURRENTPARK_H

#include "CurrentInfo.h"

#include "dds/ddsc/dds_public_impl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct byd_interfaces_msg_app_currentpark
{
  struct byd_interfaces_msg_CurrentInfo CurrentInfo;
} byd_interfaces_msg_app_currentpark;

extern const dds_topic_descriptor_t byd_interfaces_msg_app_currentpark_desc;

#define byd_interfaces_msg_app_currentpark__alloc() \
((byd_interfaces_msg_app_currentpark*) dds_alloc (sizeof (byd_interfaces_msg_app_currentpark)));

#define byd_interfaces_msg_app_currentpark_free(d,o) \
dds_sample_free ((d), &byd_interfaces_msg_app_currentpark_desc, (o))

#ifdef __cplusplus
}
#endif

#endif /* DDSC_APP_CURRENTPARK_H */
