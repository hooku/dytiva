import re
with open("m4_ver.h", "r+") as f:
    ver_str = f.read()
    ver_list = re.findall('\d+', ver_str)
    ver_list[0] = int(ver_list[0]) + 1
    f.seek(0)
    f.write("#define BUILD_NUM " + str(ver_list[0]))