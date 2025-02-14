/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  The generate program idlc named byd_idl has been modified by BYD
  File name: fusion_gridmap.h
  Source: fusion_gridmap.idl
  Cyclone DDS: V0.9.1

*****************************************************************/
#ifndef DDSC_FUSION_GRIDMAP_H
#define DDSC_FUSION_GRIDMAP_H

#include "Position.h"

#include "Freespacefusion.h"

#include "dds/ddsc/dds_public_impl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct byd_interfaces_msg_fusion_gridmap
{
  int64_t TimeStampMs;
  struct uss_msgs_msg_Position position;
  struct uss_msgs_msg_FreeSpacefusion freeSpacefusion[62500];
} byd_interfaces_msg_fusion_gridmap;

extern const dds_topic_descriptor_t byd_interfaces_msg_fusion_gridmap_desc;

#define byd_interfaces_msg_fusion_gridmap__alloc() \
((byd_interfaces_msg_fusion_gridmap*) dds_alloc (sizeof (byd_interfaces_msg_fusion_gridmap)));

#define byd_interfaces_msg_fusion_gridmap_free(d,o) \
dds_sample_free ((d), &byd_interfaces_msg_fusion_gridmap_desc, (o))

#ifdef __cplusplus
}
#endif

#endif /* DDSC_FUSION_GRIDMAP_H */
