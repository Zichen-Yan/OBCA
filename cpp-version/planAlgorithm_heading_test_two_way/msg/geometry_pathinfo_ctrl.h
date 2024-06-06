/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  The generate program idlc named byd_idl has been modified by BYD
  File name: geometry_pathinfo_ctrl.h
  Source: geometry_pathinfo_ctrl.idl
  Cyclone DDS: V0.9.1

*****************************************************************/
#ifndef DDSC_GEOMETRY_PATHINFO_CTRL_H
#define DDSC_GEOMETRY_PATHINFO_CTRL_H

#include "Coordinate.h"

#include "dds/ddsc/dds_public_impl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct byd_interfaces_msg_geometry_pathinfo_ctrl
{
  int64_t TimeStampMs;
  int32_t TrajectoryLength;
  struct uss_msgs_msg_Coordinate coordinate[300];
} byd_interfaces_msg_geometry_pathinfo_ctrl;

extern const dds_topic_descriptor_t byd_interfaces_msg_geometry_pathinfo_ctrl_desc;

#define byd_interfaces_msg_geometry_pathinfo_ctrl__alloc() \
((byd_interfaces_msg_geometry_pathinfo_ctrl*) dds_alloc (sizeof (byd_interfaces_msg_geometry_pathinfo_ctrl)));

#define byd_interfaces_msg_geometry_pathinfo_ctrl_free(d,o) \
dds_sample_free ((d), &byd_interfaces_msg_geometry_pathinfo_ctrl_desc, (o))

#ifdef __cplusplus
}
#endif

#endif /* DDSC_GEOMETRY_PATHINFO_CTRL_H */
