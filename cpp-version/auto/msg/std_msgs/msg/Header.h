/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  The generate program idlc named byd_idl has been modified by BYD
  File name: Header.h
  Source: Header.idl
  Cyclone DDS: V0.9.1

*****************************************************************/
#ifndef DDSC_HEADER_H
#define DDSC_HEADER_H

#include "builtin_interfaces/msg/Time.h"

#include "dds/ddsc/dds_public_impl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct std_msgs_msg_Header
{
  struct builtin_interfaces_msg_Time stamp;
  char * frame_id;
} std_msgs_msg_Header;

extern const dds_topic_descriptor_t std_msgs_msg_Header_desc;

#define std_msgs_msg_Header__alloc() \
((std_msgs_msg_Header*) dds_alloc (sizeof (std_msgs_msg_Header)));

#define std_msgs_msg_Header_free(d,o) \
dds_sample_free ((d), &std_msgs_msg_Header_desc, (o))

#ifdef __cplusplus
}
#endif

#endif /* DDSC_HEADER_H */
