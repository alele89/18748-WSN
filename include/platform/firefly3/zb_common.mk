
#
# Configure trace by changing ZB_TRACE_MASK (one bit per layer)
# and ZB_TRACE_LEVEL (messages with layer > ZB_TRACE_LEVEL are unvisible).
# Do not define ZB_TRACE_LEVEL to switch off trace.
#
ZB_TRACE_DEFINES = -DZB_TRACE_LEVEL=1 -DZB_TRACE_MASK=-1
#TRACE_DEFINES =

ZB_DEFINES = $(ZB_PLATFORM_DEFINES) $(ZB_TRACE_DEFINES)

EXTRAINCDIRS += $(ROOT_DIR)/include/zb_include
EXTRAINCDIRS += $(ROOT_DIR)/src/net/zb_aps
EXTRAINCDIRS += $(ROOT_DIR)/src/net/zb_common
EXTRAINCDIRS += $(ROOT_DIR)/src/net/zb_mac
EXTRAINCDIRS += $(ROOT_DIR)/src/net/zb_nwk
EXTRAINCDIRS += $(ROOT_DIR)/src/net/zb_zdo

# aps sources
SRC += $(ROOT_DIR)/src/net/zb_aps/aps_bind.c
SRC += $(ROOT_DIR)/src/net/zb_aps/aps_commands.c
SRC += $(ROOT_DIR)/src/net/zb_aps/aps_dups.c
SRC += $(ROOT_DIR)/src/net/zb_aps/aps_main.c
SRC += $(ROOT_DIR)/src/net/zb_aps/aps_nwk_confirm.c

# common sources
SRC += $(ROOT_DIR)/src/net/zb_common/zb_debug.c
SRC += $(ROOT_DIR)/src/net/zb_common/zb_scheduler.c
SRC += $(ROOT_DIR)/src/net/zb_common/zb_bufpool.c
SRC += $(ROOT_DIR)/src/net/zb_common/zb_address.c
SRC += $(ROOT_DIR)/src/net/zb_common/zb_init.c
SRC += $(ROOT_DIR)/src/net/zb_common/zb_scheduler_init.c
SRC += $(ROOT_DIR)/src/net/zb_common/zb_bufpool_init.c
SRC += $(ROOT_DIR)/src/net/zb_common/zb_time.c
SRC += $(ROOT_DIR)/src/net/zb_common/zb_ib.c
SRC += $(ROOT_DIR)/src/net/zb_common/zb_random.c
SRC += $(ROOT_DIR)/src/net/zb_common/zb_trace_firefly3.c

# mac sources
SRC += $(ROOT_DIR)/src/net/zb_mac/mac.c
SRC += $(ROOT_DIR)/src/net/zb_mac/mac_cr_coordinator.c
SRC += $(ROOT_DIR)/src/net/zb_mac/mac_fcs.c
#SRC += $(ROOT_DIR)/src/net/zb_mac/zb_uz2400.c
#SRC += $(ROOT_DIR)/src/net/zb_mac/zb_uz2410.c
SRC += $(ROOT_DIR)/src/net/zb_mac/mac_scan.c
SRC += $(ROOT_DIR)/src/net/zb_mac/mac_cr_associate.c
SRC += $(ROOT_DIR)/src/net/zb_mac/mac_associate.c
SRC += $(ROOT_DIR)/src/net/zb_mac/mac_data.c
#SRC += $(ROOT_DIR)/src/net/zb_mac/zb_ns3.c
SRC += $(ROOT_DIR)/src/net/zb_mac/mac_cr_data.c
SRC += $(ROOT_DIR)/src/net/zb_mac/mac_common.c
SRC += $(ROOT_DIR)/src/net/zb_mac/mac_routines.c
#SRC += $(ROOT_DIR)/src/net/zb_mac/zb_uz2x_common.c
SRC += $(ROOT_DIR)/src/net/zb_mac/zb_rf231_soc.c

# nwk sources
SRC += $(ROOT_DIR)/src/net/zb_nwk/nwk_main.c
SRC += $(ROOT_DIR)/src/net/zb_nwk/nwk_nlme.c
SRC += $(ROOT_DIR)/src/net/zb_nwk/nwk_discovery.c
SRC += $(ROOT_DIR)/src/net/zb_nwk/nwk_formation.c
SRC += $(ROOT_DIR)/src/net/zb_nwk/nwk_cr_formation.c
SRC += $(ROOT_DIR)/src/net/zb_nwk/nwk_cr_permit_join.c
SRC += $(ROOT_DIR)/src/net/zb_nwk/nwk_cr_join.c
SRC += $(ROOT_DIR)/src/net/zb_nwk/nwk_join.c
SRC += $(ROOT_DIR)/src/net/zb_nwk/nwk_neighbor.c
SRC += $(ROOT_DIR)/src/net/zb_nwk/nwk_cr_route_discovery.c
SRC += $(ROOT_DIR)/src/net/zb_nwk/nwk_address_assign.c
SRC += $(ROOT_DIR)/src/net/zb_nwk/nwk_cr_tree_routing.c
SRC += $(ROOT_DIR)/src/net/zb_nwk/nwk_cr_mesh_routing.c
SRC += $(ROOT_DIR)/src/net/zb_nwk/nwk_panid_conflict.c

# zdo sources
SRC += $(ROOT_DIR)/src/net/zb_zdo/zdo_app.c
SRC += $(ROOT_DIR)/src/net/zb_zdo/zdo_rx.c
SRC += $(ROOT_DIR)/src/net/zb_zdo/zdo_disc_cli.c
SRC += $(ROOT_DIR)/src/net/zb_zdo/af_rx.c
SRC += $(ROOT_DIR)/src/net/zb_zdo/zdo_disc_srv.c
SRC += $(ROOT_DIR)/src/net/zb_zdo/af_descriptor.c
SRC += $(ROOT_DIR)/src/net/zb_zdo/zdo_common.c
SRC += $(ROOT_DIR)/src/net/zb_zdo/zdo_nwk_manage_cli.c
SRC += $(ROOT_DIR)/src/net/zb_zdo/zdo_nwk_manage_srv.c
SRC += $(ROOT_DIR)/src/net/zb_zdo/zdo_channel_manager.c
SRC += $(ROOT_DIR)/src/net/zb_zdo/test_profile.c
SRC += $(ROOT_DIR)/src/net/zb_zdo/zdo_bind_manage.c
