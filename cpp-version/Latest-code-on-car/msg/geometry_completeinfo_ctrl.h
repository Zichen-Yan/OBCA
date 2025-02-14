/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  The generate program idlc named byd_idl has been modified by BYD
  File name: geometry_completeinfo_ctrl.h
  Source: geometry_completeinfo_ctrl.idl
  Cyclone DDS: V0.9.1

*****************************************************************/
#ifndef DDSC_GEOMETRY_COMPLETEINFO_CTRL_H
#define DDSC_GEOMETRY_COMPLETEINFO_CTRL_H

#include "dds/ddsc/dds_public_impl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct byd_interfaces_msg_geometry_completeinfo_ctrl
{
  int64_t TimeStampMs;
  uint8_t IsPlanningCompleted;
} byd_interfaces_msg_geometry_completeinfo_ctrl;

extern const dds_topic_descriptor_t byd_interfaces_msg_geometry_completeinfo_ctrl_desc;

#define byd_interfaces_msg_geometry_completeinfo_ctrl__alloc() \
((byd_interfaces_msg_geometry_completeinfo_ctrl*) dds_alloc (sizeof (byd_interfaces_msg_geometry_completeinfo_ctrl)));

#define byd_interfaces_msg_geometry_completeinfo_ctrl_free(d,o) \
dds_sample_free ((d), &byd_interfaces_msg_geometry_completeinfo_ctrl_desc, (o))

#ifdef __cplusplus
}
#endif

#endif /* DDSC_GEOMETRY_COMPLETEINFO_CTRL_H */
