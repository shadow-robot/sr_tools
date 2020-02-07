#!/usr/bin/env python

# Copyright (C) 2019 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
import logging
import boto3
from botocore.exceptions import ClientError


# def upload_file(file_name, bucket, object_name=None):
#     """Upload a file to an S3 bucket

#     :param file_name: File to upload
#     :param bucket: Bucket to upload to
#     :param object_name: S3 object name. If not specified then file_name is used
#     :return: True if file was uploaded, else False
#     """

#     # If S3 object_name was not specified, use file_name
#     if object_name is None:
#         object_name = file_name

#     # Upload the file
#     s3_client = boto3.client('s3')
#     try:
#         response = s3_client.upload_file(file_name, bucket, object_name)
#     except ClientError as e:
#         logging.error(e)
#         return False
#     return True

if __name__ == "__main__":
    rospy.init_node("aws_manager_node")

    download_param = rospy.get_param("~download")
    upload_param = rospy.get_param("~upload")
    file_name = rospy.get_param("~file_name")

    s3 = boto3.client('s3')
    #bucket = s3.Bucket("shadowrobot.healthreport.results")

    if upload_param is True:
        with open(file_name, "rb") as upload_filename:
            print(upload_filename)
            print(type(upload_filename))
            s3.upload_fileobj(upload_filename, "shadowrobot.healthreport.results", "{}".format("2346"))
    if download_param is True:
        with open(file_name, 'wb') as download_filename:
            s3.download_file("shadowrobot.healthreport.results", "health_report_results_test.yml", "health_report_results_test.yml")
