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
import os

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

    try:
        with open('/usr/local/bin/customer.key', 'r') as customer_key_file:
            customer_key = customer_key_file.read()
    except:
        rospy.logerr("Could not find customer key, ask software team for help!")

    headers = {
        'x-api-key': 'miZSxsc8ud32F9sLlDBS7Co5eRQIeZ18B0bezvTf',
    }

    try:
        response = requests.get('https://5vv2z6j3a7.execute-api.eu-west-2.amazonaws.com/prod', headers=headers)
    except:
        rospy.logerr("Could request secret AWS access key, ask software team for help!")

    result = re.search('ACCESS_KEY_ID=(.*)\nSECRET_ACCESS', response.text)
    aws_access_key_id = result.group(1)

    result = re.search('SECRET_ACCESS_KEY=(.*)\nSESSION_TOKEN', response.text)
    aws_secret_access_key = result.group(1)

    result = re.search('SESSION_TOKEN=(.*)\nEXPIRATION', response.text)
    aws_session_token = result.group(1)

    client = boto3.client(
        's3',
        aws_access_key_id=aws_access_key_id,
        aws_secret_access_key=aws_secret_access_key,
        aws_session_token=aws_session_token
    )

    bucket_name = "shadowrobot.healthreport.results"
    if upload_param is True:
        rospy.loginfo("Uploading report yaml file..")
        client.upload_file(full_report_path, bucket_name, aws_report_path)
        rospy.loginfo("Upload download of report yaml file!")

        rospy.loginfo("Uploading bag file..")
        client.upload_file(full_bag_path, bucket_name, aws_bag_path)
        rospy.loginfo("Completed Upload of bag file!")

        rospy.loginfo("Uploading param dump yaml file..")
        client.upload_file(full_yaml_dump_path, bucket_name, aws_yaml_dump_path)
        rospy.loginfo("Completed Upload of param dump yaml file!")

    if download_param is True:
        directory = "{}/{}".format(file_path, folder_path)
        if not os.path.exists(directory):
            os.makedirs(directory)
        rospy.loginfo("Downloading report yaml file..")
        client.download_file(bucket_name, aws_report_path, "{}/{}/{}".format(file_path, folder_path, report_file_name))
        rospy.loginfo("Completed download of report yaml file!")

        rospy.loginfo("Downloading bag file..")
        client.download_file(bucket_name, aws_bag_path, "{}/{}/{}".format(file_path, folder_path, rosbag_file_name))
        rospy.loginfo("Completed download of bag file!")

        rospy.loginfo("Downloading param dump yaml file..")
        client.download_file(bucket_name, aws_yaml_dump_path, "{}/{}/{}".format(file_path, folder_path,
                                                                                param_dump_file_name))
        rospy.loginfo("Completed download of param dump yaml file!")

    rospy.signal_shutdown("")
