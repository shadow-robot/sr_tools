#!/usr/bin/env python
# Copyright (C) 2020 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
import logging
import boto3
from botocore.exceptions import ClientError
if __name__ == "__main__":
    rospy.init_node("aws_manager_node")
    download_param = rospy.get_param("~download")
    upload_param = rospy.get_param("~upload")
    file_name = rospy.get_param("~file_name")
    s3_resource = boto3.resource('s3')
    bucket_name = "shadowrobot.healthreport.resutls"
    if upload_param is True:
        with open(file_name, "r") as upload_filename:
            s3_resource.Bucket(bucket_name).upload_file(
                       Filename=upload_filename, Key=upload_filename)
    if download_param is True:
        s3_resource.Object(bucket_name, file_name).download_file("/tmp/{}".format(file_name))