from esp32 import NVS

__all__ = ("Settings",)


class Settings:
    def __init__(self, namespace):
        self._nvs = NVS(namespace)

    def commit(self):
        self._nvs.commit()

    def erase(self, key):
        self._nvs.erase_key(key)

    def get_int(self, key):
        return self._nvs.get_i32(key)

    def set_int(self, key, value, commit=False):
        self._nvs.set_i32(key, value)
        if commit:
            self.commit()

    def get_bool(self, key):
        return bool(self.get_int(key))

    def set_bool(self, key, value, commit=False):
        value = 1 if value else 0
        self.set_int(key, value, commit)

    def get_blob(self, key):
        buf = bytearray()
        self._nvs.get_blob(key, buf)
        return buf

    def set_blob(self, key, value, commit=False):
        self._nvs.set_blob(key, value)
        if commit:
            self.commit()

    def get_str(self, key, decode="utf-8"):
        buf = bytearray()
        size = self._nvs.get_blob(key, buf)
        return buf[0:size].decode(decode)

    def set_str(self, key, value, commit=False):
        self.set_blob(key, value, commit)

    def get_float(self, key):
        return float(self.get_str(key))

    def set_float(self, key, value, commit=False):
        self.set_str(key, str(value))
        if commit:
            self.commit()
