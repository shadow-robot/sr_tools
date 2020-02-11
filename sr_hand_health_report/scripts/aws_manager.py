#!/usr/bin/env python
# Copyright (C) 2020 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
import logging
import boto3
from botocore.exceptions import ClientError
import subprocess
import requests
import re
import json


if __name__ == "__main__":
    rospy.init_node("aws_manager_node")

    download_param = rospy.get_param("~download")
    upload_param = rospy.get_param("~upload")

    file_path = rospy.get_param("~file_path")
    folder_path = rospy.get_param("~folder_path")
    report_file_name = rospy.get_param("~report_name")
    rosbag_file_name = rospy.get_param("~bag_name")
    param_dump_file_name = rospy.get_param("~param_dump_file_name")

    full_report_path = "{}/{}/{}".format(file_path, folder_path, report_file_name)
    full_bag_path = "{}/{}/{}".format(file_path, folder_path, rosbag_file_name)
    full_yaml_dump_path = "{}/{}/{}".format(file_path, folder_path, param_dump_file_name)

    aws_report_path = "{}/{}".format(folder_path, report_file_name)
    aws_bag_path = "{}/{}".format(folder_path, rosbag_file_name)
    aws_yaml_dump_path = "{}/{}".format(folder_path, param_dump_file_name)

    headers = {
        'x-api-key': 'miZSxsc8ud32F9sLlDBS7Co5eRQIeZ18B0bezvTf',
    }
    response = requests.get('https://5vv2z6j3a7.execute-api.eu-west-2.amazonaws.com/prod', headers=headers)
    
    result = re.search('ACCESS_KEY_ID=(.*)\nSECRET_ACCESS', response.text)
    aws_access_key_id = result.group(1)

    result = re.search('SECRET_ACCESS_KEY=(.*)\nSESSION_TOKEN', response.text)
    aws_secret_access_key = result.group(1)

    result = re.search('SESSION_TOKEN=(.*)\nEXPIRATION', response.text)
    aws_session_token = result.group(1)

    client = boto3.client(
        's3',
        # Hard coded strings as credentials, not recommended.
        aws_access_key_id=aws_access_key_id,
        aws_secret_access_key=aws_secret_access_key,
        aws_session_token=aws_session_token
    )

    #s3_resource = boto3.resource('s3')
    bucket_name = "shadowrobot.healthreport.results"
    if upload_param is True:
        client.upload_file(full_report_path, bucket_name, aws_report_path)
        client.upload_file(full_bag_path, bucket_name, aws_bag_path)
        client.upload_file(full_yaml_dump_path, bucket_name, aws_yaml_dump_path)
    if download_param is True:
        s3_resource.Object(bucket_name, file_name).download_file("/tmp/{}".format(file_name))
