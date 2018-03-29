#ifndef __PANFROST_H__
#define __PANFROST_H__

#include <drm/drmP.h>

/* legacy definitions */
#include "mali_kbase_defs.h"

struct panfrost_device {
	struct device *dev;
	struct drm_device *ddev;
	void __iomem *iomem;

	/* legacy mali vendor driver struct */
	struct kbase_device kbdev;
};


#endif
