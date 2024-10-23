from io import BytesIO
from urllib.request import urlopen
from zipfile import ZipFile
import os
import subprocess
import shutil
import argparse


def arg_parse():
    parser = argparse.ArgumentParser(description="Sensoris Data Transmission.")
    parser.add_argument('-su', "--sensoris-zip-url", required=False,
                        default="https://sensoris.org/wp-content/uploads/sites/21/2022/04/sensoris-specification-v1.3.1-1.zip",
                        type=str,
                        help='URL of Sensoris zip file')

    parser.add_argument('-dl', "--download", required=False, default=False, action='store_true',
                        help='Download Sensoris zip file or not')

    args = parser.parse_args()

    return args


def sensoris_prep(sensoris_zip_url = "https://sensoris.org/wp-content/uploads/sites/21/2022/04/sensoris-specification-v1.3.1-1.zip",
                  sensoris_dir="./sensoris_orig", download=False):
    """
    Download SENSORIS from URL as option, and build the pb2 package for Pysensoris
    :param sensoris_zip_url: SENSORIS download url
    :param sensoris_dir: target sensoris pb2 dir
    :param download: download True or False
    """
    tem_pb_dir = "proto_python"
    args = arg_parse()
    sensoris_zip_url = args.sensoris_zip_url
    download = args.download

    def clear_sensoris_dir(start=True):
        if os.path.exists(sensoris_dir):
            shutil.rmtree(sensoris_dir)

        if os.path.exists('sensoris'):
            shutil.rmtree('sensoris')

        if not start:
            destination = shutil.copytree('proto_python/sensoris', 'sensoris')
        if os.path.exists(tem_pb_dir):
            shutil.rmtree(tem_pb_dir)

        if start:
            if not os.path.exists(sensoris_dir):
                os.makedirs(sensoris_dir)

            if not os.path.exists(tem_pb_dir):
                os.makedirs(tem_pb_dir)

    clear_sensoris_dir()
    if download:

        with urlopen(sensoris_zip_url) as zipresp:
            with ZipFile(BytesIO(zipresp.read())) as zfile:
                zfile.extractall(sensoris_dir)

    else:
        print('The Sensoris zip file should be downloaded manually')
        if not os.path.exists(sensoris_dir):
            os.makedirs(sensoris_dir)
        with ZipFile('sensoris-specification-v1.3.1-1.zip', 'r') as zip:
            zip.extractall(sensoris_dir)


    destination = shutil.copytree(os.path.join(sensoris_dir, 'sensoris-specification-v1.3.1/src/sensoris'), 'sensoris')
    subprocess.call(['sh', './protogen.sh'])

    clear_sensoris_dir(False)


if __name__ == "__main__":
    sensoris_prep()





