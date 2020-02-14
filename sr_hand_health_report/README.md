## SR_HAND_HEALTH_REPORT PACKAGE

Package that contains code to run Automatic Health Report tests.

The purpose of the script will be to collect information on the health of the hand at time of manufacture. Production and Clients will then be able to run this script when they suspect a fault or degraded performance of the hand. The script will generate a health report with checks, to investigate possible causes of fault.

### Usage

#### Get AWS Access Key

This step is necessary in order to be able to upload and download results to AWS.

To get the AWS Access Key you need to install your container by running the following command:
```
bash <(curl -Ls bit.ly/run-aurora) docker_deploy --read-secure customer_key use_aws=true product=hand_e ethercat_interface=enp5s0 config_branch=demohand_D nvidia_docker=true reinstall=true tag=melodic-release image=shadowrobot/dexterous-hand
```

During installation you will be prompted for a AWS customer key. This key can be retrieved from [here](http://10.5.1.13/mediawiki/index.php/Customer_Keys_for_uploading_ROS_Logs), copy and paste one from the Table of Customer Keys.

If you already have a container installed which does not contain an AWS key, retrieve one of the keys from the link above and within your container run the following command:

```
echo "your_aws_customer_key" | sudo tee /usr/local/bin/customer_key.key
```

If you have doubts about this process contact the software team.


#### Run the hand

In order to run the script, first run the hand with the following command:

```
roslaunch sr_hand_health_report sr_hand.launch
```

This launch file will run the driver and load the required PWM controllers.

Before starting the test make sure the hand is in a position similar to the one showed in the picture below:

![Hand Pose](https://github.com/shadow-robot/sr_tools/blob/F%23SRC-3740_health_report_script/sr_hand_health_report/images/health_report_hand.png)
In order to move the hand in the wanted position you can run it in Teach Mode by following the instructions here:

In the terminal execute the following command:

```
rqt
```

On the top-left bar select **Plugins->ShadowRobot->ChangeControllers**.
In the window that will show up select **Teach Mode** for the hand under test as shown in the picture below.

![Teach Mode Image](https://github.com/shadow-robot/sr_tools/blob/F%23SRC-3740_health_report_script/sr_hand_health_report/images/teach_mode_image.png)

#### Health report launch file

The launch file to run the health report checks and generate the report is called [sr_hand_health_report.launch](https://github.com/shadow-robot/sr_tools/tree/F%23SRC-3740_health_report_script/sr_hand_health_report/launch).


The launch file allows you to specify which side of the hand is under test (left or right), which checks you want to execute and for which finger.

The available tests are:
 - monotonicity_check
 - positions_sensor_noise_check

To run all the checks for all the fingers run the following command:

```
roslaunch sr_hand_health_report sr_hand_health_report.launch hand_side:=<hand_side>
```


If, for instance, you only want to run the *position_sensor_noise_check* for First finger and Middle Finger, you can run the following command:

```
roslaunch sr_hand_health_report sr_hand_health_report.launch hand_side:=<hand_side> checks_to_run:="[position_sensor_noise_check]" fingers_to_test="[FF, MF]"
```

#### Data visualization and Upload

To access the reports execute the followin command:

```
cd /home/user/sr_hand_health_reports
```

This folder will contain a folder per hand serial number, which will contain a folder with the date and time at which the check was executed. Each test folder contains the **health_report_result.yml**, the **health_report_bag_file.bag** and the **param_dump.yaml** file.

If you want to upload a certain folder to AWS run the following command:

```
roslaunch sr_log_aws_files.launch upload:=true hand_serial:=<hand_serial> test_date:=<test_date> test_time:=<test_time>
```

This will upload the test result to the [shadowrobot.healthreport.results bucket](https://s3.console.aws.amazon.com/s3/buckets/shadowrobot.healthreport.results/?region=eu-west-2&tab=overview).


#### Data Download

To download a specific test result run the following command:

```
roslaunch sr_log_aws_files.launch download:=true hand_serial:=<hand_serial> test_date:=<test_date> test_time:=<test_time>
```

This will download a given report to your home folder.


#### FOR SOFTWARE: Run tests on bag files

To run a test on a bag file, run the following command:

```
roslaunch sr_hand_health_report sr_hand_health_report.launch hand_side:=<hand_side> real_hand:=false
hand_serial:=<hand_serial> test_date:=<test_date> test_time:=<test_time>
```

This will automatically play a rosbag and run the wanted checks on it.
