/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  The generate program idlc named byd_idl has been modified by BYD
  File name: head.h
  Source: head.idl
  Cyclone DDS: V0.9.1

*****************************************************************/
#ifndef DDSC_HEAD_H
#define DDSC_HEAD_H

#include "dds/ddsc/dds_public_impl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct uss_msgs_msg_Head
{
  int32_t x;
  int32_t y;
  double Heading;
} uss_msgs_msg_Head;

extern const dds_topic_descriptor_t uss_msgs_msg_Head_desc;

#define uss_msgs_msg_Head__alloc() \
((uss_msgs_msg_Head*) dds_alloc (sizeof (uss_msgs_msg_Head)));

#define uss_msgs_msg_Head_free(d,o) \
dds_sample_free ((d), &uss_msgs_msg_Head_desc, (o))

#ifdef __cplusplus
}
#endif

#endif /* DDSC_HEAD_H */
