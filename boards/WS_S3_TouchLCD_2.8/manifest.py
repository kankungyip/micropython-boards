include("$(PORT_DIR)/boards")

include("$(MOD_DIR)/fos")
include("$(MOD_DIR)/blerepl")

include("$(MOD_DIR)/utils/manifest.macutils.py")
include("$(MOD_DIR)/utils/manifest.settings.py")

include("$(MPY_DIR)/../lv_binding/ports/esp32")
include("$(MOD_DIR)/lv_drivers/manifest.display.st77xx.py")
include("$(MOD_DIR)/lv_drivers/manifest.touch.cst3xx.py")

require("base64")
require("hmac")
require("logging")
require("threading")

require("aioble")
require("aioespnow")
require("aiohttp")
require("ntptime")
require("umqtt.simple")

freeze("modules")
