/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:
 *      Sandy Huang <hjc@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_panel.h>
#include <drm/drm_of.h>

#include <linux/component.h>
#include <linux/of_graph.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_vop.h"

#define encoder_to_rgb(c) container_of(c, struct rockchip_rgb, encoder)

struct rockchip_rgb {
	struct device *dev;
	struct drm_device *drm_dev;
	struct drm_bridge *bridge;
	struct drm_encoder encoder;
//	struct dev_pin_info *pins;
	int output_mode;
};

static inline int name_to_output_mode(const char *s)
{
	static const struct {
		const char *name;
		int format;
	} formats[] = {
		{ "p888", ROCKCHIP_OUT_MODE_P888 },
		{ "p666", ROCKCHIP_OUT_MODE_P666 },
		{ "p565", ROCKCHIP_OUT_MODE_P565 }
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(formats); i++)
		if (!strncmp(s, formats[i].name, strlen(formats[i].name)))
			return formats[i].format;

	return -EINVAL;
}

static void rockchip_rgb_encoder_enable(struct drm_encoder *encoder)
{
	struct rockchip_rgb *rgb = encoder_to_rgb(encoder);

//	if (rgb->pins && !IS_ERR(rgb->pins->default_state))
//		pinctrl_select_state(rgb->pins->p, rgb->pins->default_state);
}

static void rockchip_rgb_encoder_disable(struct drm_encoder *encoder)
{
	struct rockchip_rgb *rgb = encoder_to_rgb(encoder);

//FIXME: pin-voodoo?
}

static int
rockchip_rgb_encoder_atomic_check(struct drm_encoder *encoder,
				   struct drm_crtc_state *crtc_state,
				   struct drm_connector_state *conn_state)
{
	struct rockchip_crtc_state *s = to_rockchip_crtc_state(crtc_state);
	struct rockchip_rgb *rgb = encoder_to_rgb(encoder);

	s->output_mode = rgb->output_mode;
	s->output_type = DRM_MODE_CONNECTOR_LVDS;

	return 0;
}

static const
struct drm_encoder_helper_funcs rockchip_rgb_encoder_helper_funcs = {
	.enable = rockchip_rgb_encoder_enable,
	.disable = rockchip_rgb_encoder_disable,
	.atomic_check = rockchip_rgb_encoder_atomic_check,
};

static const struct drm_encoder_funcs rockchip_rgb_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

struct rockchip_rgb *rockchip_rgb_init(struct device *dev,
				       struct drm_crtc *crtc,
				       struct drm_device *drm_dev)
{
	struct rockchip_rgb *rgb;
	struct drm_encoder *encoder;
	struct device_node *port, *endpoint;
	u32 endpoint_id;
	int ret = 0, child_count = 0;
	struct drm_panel *panel;
	struct drm_bridge *bridge;

	rgb = devm_kzalloc(dev, sizeof(*rgb), GFP_KERNEL);
	if (!rgb)
		return ERR_PTR(-ENOMEM);

	rgb->dev = dev;
	rgb->drm_dev = drm_dev;

/*	rgb->pins = devm_kzalloc(rgb->dev, sizeof(*rgb->pins), GFP_KERNEL);
	if (!rgb->pins)
		return -ENOMEM; */

/*	rgb->pins->p = devm_pinctrl_get(rgb->dev);
	if (IS_ERR(rgb->pins->p)) {
		DRM_DEV_ERROR(dev, "no pinctrl handle\n");
		devm_kfree(rgb->dev, rgb->pins);
		rgb->pins = NULL;
	} else {
		rgb->pins->default_state =
			pinctrl_lookup_state(rgb->pins->p, "lcdc");
		if (IS_ERR(rgb->pins->default_state)) {
			DRM_DEV_ERROR(dev, "no default pinctrl state\n");
			devm_kfree(rgb->dev, rgb->pins);
			rgb->pins = NULL;
		}
	} */

	port = of_graph_get_port_by_id(dev->of_node, 0);
	if (!port)
		return ERR_PTR(-EINVAL);

	for_each_child_of_node(port, endpoint) {
		if (of_property_read_u32(endpoint, "reg", &endpoint_id))
			endpoint_id = 0;

		if (rockchip_drm_endpoint_is_subdriver(endpoint))
			continue;

		child_count++;
		ret = drm_of_find_panel_or_bridge(dev->of_node, 0, endpoint_id,
						  &panel, &bridge);
		if (!ret)
			break;
	}

	of_node_put(port);

	/* if the rgb output is not connected to anything, just return */
	if (!child_count)
		return NULL;

	if (ret < 0) {
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev, "failed to find panel or bridge %d\n", ret);
		return ERR_PTR(ret);
	}

	rgb->output_mode = ROCKCHIP_OUT_MODE_P888;

	encoder = &rgb->encoder;
	encoder->possible_crtcs = drm_crtc_mask(crtc);

	ret = drm_encoder_init(drm_dev, encoder, &rockchip_rgb_encoder_funcs,
			       DRM_MODE_ENCODER_NONE, NULL);
	if (ret < 0) {
		DRM_DEV_ERROR(drm_dev->dev,
			      "failed to initialize encoder: %d\n", ret);
		return ERR_PTR(ret);
	}

	drm_encoder_helper_add(encoder, &rockchip_rgb_encoder_helper_funcs);

	if (panel) {
		bridge = drm_panel_bridge_add(panel, DRM_MODE_CONNECTOR_LVDS);
		if (IS_ERR(bridge))
			return ERR_CAST(bridge);
	}

	rgb->bridge = bridge;

	ret = drm_bridge_attach(encoder, rgb->bridge, NULL);
	if (ret) {
		DRM_DEV_ERROR(drm_dev->dev,
			      "failed to attach bridge: %d\n", ret);
		goto err_free_encoder;
	}

	return rgb;

err_free_encoder:
	drm_encoder_cleanup(encoder);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(rockchip_rgb_init);

void rockchip_rgb_fini(struct rockchip_rgb *rgb)
{
	rockchip_rgb_encoder_disable(&rgb->encoder);

	drm_panel_bridge_remove(rgb->bridge);
	drm_encoder_cleanup(&rgb->encoder);
}
EXPORT_SYMBOL_GPL(rockchip_rgb_fini);
