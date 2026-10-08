include("$(PORT_DIR)/boards")

include("$(MOD_DIR)/fos")
include("$(MOD_DIR)/blerepl")

include("$(MOD_DIR)/utils/macutils.manifest.py")
include("$(MOD_DIR)/utils/settings.manifest.py")

include("$(MPY_DIR)/../lv_binding/ports/esp32")
include("$(MOD_DIR)/lv_drivers/display/ili9xx.manifest.py")
include("$(MOD_DIR)/lv_drivers/touch/cst3xx.manifest.py")
include("$(MOD_DIR)/lv_drivers/touch/cst8xx.manifest.py")
include("$(MOD_DIR)/lv_drivers/touch/ft6x36.manifest.py")
include("$(MOD_DIR)/lv_drivers/touch/xpt2046.manifest.py")

require("base64")
require("hmac")
require("logging")
require("threading")

require("aioble")
require("aioespnow")
require("aiohttp")
require("ntptime")
require("umqtt.simple")
