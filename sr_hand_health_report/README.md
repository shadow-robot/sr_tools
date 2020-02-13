## SR_HAND_HEALTH_REPORT PACKAGE

Package that contains code to run Automatic Health Report tests.

The purpose of the script will be to collect information on the health of the hand at time of manufacture. Production and Clients will then be able to run this script when they suspect a fault or degraded performance of the hand. The script will generate a health report with checks, to investigate possible causes of fault.

# Usage

In order to run the script, first run the hand with the following command:

```
roslaunch sr_hand_health_report sr_hand.launch
```

After RViz has loaded correctly you can finally run the health report script, by running the following command:

```
roslaunch sr_hand_health_report sr_hand_health_report.launch hand_side:=<hand_side>
```

The hand_side argument needs to be specified in order to know whether the hand under test is a right or left hand.

The launch file allows you to specify which tests you want to run.

The available tests are:
 - monotonicity_check
 - positions_sensor_noise_check

By default all the test are performed, but if, for instance, you only want to run the position_sensor_noise_check you can run the following command:

```
roslaunch sr_hand_health_report sr_hand_health_report.launch hand_side:=<hand_side> checks_to_run:="[position_sensor_noise_check]" fingers_to_test="[FF, MF]"
```

# Get AWS Access Key

To get the AWS Access Key you need to install your container by following the instructions for running a container here.
The option aws needs to be set equal to True.
The script will ask you for an access key during installation. This key can be retrieved for here.

If you already have a container, retrieve one of the keys and within your container run the following command:

```
cat /customer.key
```

If you have doubts about this process contact the software team.

# Data visualization and Upload

To access the reports execute the followin command:

```
cd /home/user/sr_hand_health_reports
```

This folder will contain a folder per hand serial number, which will contain a folder with the date and time at which the check was executed. The folder structure is shown for clarity in the picture below:


If you want to upload a certain folder to AWS run the following command:

```
roslaunch sr_log_aws_files.launch upload:=true hand_serial:=<hand_serial> test_date:=<test_date> test_time:=<test_time>
```

This will upload the test result to and AWS bucket as shown in the picture below:
