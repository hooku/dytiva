#!/usr/bin/env python
# === auto config file by hooku, v1 ===
import os, shutil, sys
import zipfile
if sys.version_info[0] >= 3:
    from urllib.request import urlretrieve
else:
    from urllib import urlretrieve

REPO_TMP = '_repo'

def get_repo(url, extract_dir, dst_dir):
    current_path = os.path.dirname(os.path.realpath(__file__))
    repo_tmp_path = os.path.join(current_path, REPO_TMP)
    extract_path = os.path.join(repo_tmp_path, extract_dir)
    dst_path = os.path.join(current_path, dst_dir)
    repo_tmp_file = os.path.join(current_path, REPO_TMP + '.tmp')
    
    print("Download:", url)
    urlretrieve(url, repo_tmp_file)
    
    zip_ref = zipfile.ZipFile(repo_tmp_file, 'r')
    for zip_file in zip_ref.namelist():
        if zip_file.startswith(extract_dir):
            print("Extract:", zip_file)
            zip_ref.extract(zip_file, repo_tmp_path)
    zip_ref.close()

    if not os.path.exists(dst_path):
        os.makedirs(dst_path)
        
    for dirpath, dirnames, filenames in os.walk(extract_path):
        relative_path = os.path.relpath(dirpath, extract_path)
        extract_sub_path = os.path.join(extract_path, relative_path)
        dst_sub_path = os.path.join(dst_path, relative_path)
        if not os.path.exists(dst_sub_path):
            os.makedirs(dst_sub_path)
        for file in filenames:
            extract_file = os.path.join(extract_sub_path, file)
            dst_file = os.path.join(dst_sub_path, file)
            if not os.path.exists(dst_file):
                print(dst_file)
                os.rename(extract_file, dst_file)
            else:
                print(dst_file, "already exist")
        
    shutil.rmtree(repo_tmp_path, ignore_errors=True)
    os.remove(repo_tmp_file)

get_repo("http://elm-chan.org/fsw/ff/arc/ff10a.zip", 'src/', 'M4MP3/FatFs/')
get_repo("https://storage.googleapis.com/google-code-archive-downloads/v2/code.google.com/m2tklib/m2tklib_arduino_u8g_1.11.zip", 'M2tklib/utility/', 'M4MP3/m2tklib/')
get_repo("https://bintray.com/olikraus/u8glib/download_file?file_path=u8glib_arm_v1.15.zip", 'src/', 'M4MP3/u8glib/')
