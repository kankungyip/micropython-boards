import random

__all__ = ("shorten", "mac_bytes", "mac_str")

_CHAR_TABLE = "0123456789ABCDEF"


def _normalize_mac(mac_str):
    """标准化MAC地址"""
    result = bytearray(12)
    idx = 0
    for c in mac_str:
        if idx >= 12:
            break
        if "0" <= c <= "9":
            result[idx] = ord(c)
            idx += 1
        elif "A" <= c <= "F":
            result[idx] = ord(c)
            idx += 1
        elif "a" <= c <= "f":
            result[idx] = ord(c.upper())
            idx += 1
    return result.decode() if idx == 12 else None


def _mac_to_int(mac_clean):
    """MAC地址转整数（优化版）"""
    value = 0
    for i in range(0, 12, 2):
        high = ord(mac_clean[i]) - (48 if mac_clean[i] <= "9" else 55)
        low = ord(mac_clean[i + 1]) - (48 if mac_clean[i + 1] <= "9" else 55)
        value = (value << 8) | ((high << 4) | low)
    return value


def _simple_hash(mac_int, seed):
    """简单但有效的哈希函数"""
    h = mac_int
    h = (h * 0x5DEECE66D) & 0xFFFFFFFFFFFF  # 2^48-1
    h ^= seed
    h = ((h << 13) & 0xFFFFFFFFFFFF) | (h >> (48 - 13))
    h ^= 0x5A5A5A5A5A5A & 0xFFFFFFFFFFFF
    return h & 0xFFFFFFFFFFFF


def _int_to_base16(num, length):
    """整数转Base16（优化版）"""
    result = ["0"] * length
    for i in range(length - 1, -1, -1):
        num, remainder = divmod(num, 16)
        result[i] = _CHAR_TABLE[remainder]
    return "".join(result)


def _mix_with_fibonacci(h):
    """使用斐波那契数列混合，确保更好的分布"""
    h = (h ^ (h >> 17)) & 0xFFFFFFFFFFFF
    h = (h * 0xED5AD4BB) & 0xFFFFFFFFFFFF
    h = (h ^ (h >> 11)) & 0xFFFFFFFFFFFF
    h = (h * 0xAC4C1B51) & 0xFFFFFFFFFFFF
    h = (h ^ (h >> 15)) & 0xFFFFFFFFFFFF
    h = (h * 0x31848BAB) & 0xFFFFFFFFFFFF
    h = (h ^ (h >> 14)) & 0xFFFFFFFFFFFF
    return h & 0xFFFFFFFFFFFF


def shorten(mac_str, max_length=6, seed=None):
    """将MAC地址缩短为6位代码"""
    mac_clean = _normalize_mac(mac_str)
    if mac_clean is None:
        raise ValueError("Invalid MAC address format")
    seed = seed if seed is not None else random.getrandbits(16)
    mac_int = _mac_to_int(mac_clean)
    hashed = _simple_hash(mac_int, seed)
    hashed = _mix_with_fibonacci(hashed)
    short_mac = _int_to_base16(hashed, max_length)
    return short_mac


def mac_bytes(mac_str):
    return bytes.fromhex(mac_str.replace(":", ""))


def mac_str(mac_bytes, sep=":"):
    return sep.join([f"{b:02x}" for b in mac_bytes])
