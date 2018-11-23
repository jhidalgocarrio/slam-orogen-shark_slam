#!/usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'rock/bundle'
require 'vizkit'
require 'utilrb'
require 'optparse'

include Orocos

options = {}

op = OptionParser.new do |opt|
    opt.banner = <<-EOD
    usage: process_logs_shark_slam [options] <data_log_directory>
    EOD
    opt.on '--help', 'this help message' do
        puts opt
       exit 0
    end
end

args = op.parse(ARGV)
logfiles_path = args.shift

if !logfiles_path
    puts "missing path to log files"
    puts options
    exit 1
end

Orocos::CORBA::max_message_size = 100000000000
Rock::Bundles.initialize
Rock::Bundles.transformer.load_conf(Orocos.find_file('config', 'transforms_scripts_shark_slam.rb'))

Orocos.run 'exoter_control',
            'shark_slam::Task' => 'shark_slam',
            'localization_frontend::Task' => 'localization_frontend',
            :gdb => false do

    # Get the task names for the shark_slam
    shark_slam = Orocos.name_service.get 'shark_slam'
    read_joint_dispatcher = Orocos.name_service.get 'read_joint_dispatcher'
    ptu_control = Orocos.name_service.get 'ptu_control'
    localization_frontend = Orocos.name_service.get 'localization_frontend'

    # Set configuration files for control
    Orocos.conf.apply(shark_slam, ['default'], :override => true)
    Orocos.conf.apply(read_joint_dispatcher, ['reading'], :override => true)
    Orocos.conf.apply(ptu_control, ['default'], :override => true)
    Orocos.conf.apply(localization_frontend, ['default', 'hamming1hzsampling12hz'], :override => true)
    localization_frontend.urdf_file = Orocos.find_file('data/odometry', 'exoter_odometry_model_complete.urdf')

    # logs files
    log_replay = Orocos::Log::Replay.open( logfiles_path )

    #################
    ## TRANSFORMER ##
    #################
    Orocos.transformer.setup(shark_slam)
    Orocos.transformer.setup(localization_frontend)

    ###################
    ## LOG THE PORTS ##
    ###################
    Orocos.log_all

    # Configure tasks for shark_slam
    shark_slam.configure
    read_joint_dispatcher.configure
    ptu_control.configure
    localization_frontend.configure

    ###########################
    ## LOG PORTS CONNECTIONS ##
    ###########################
    log_replay.platform_driver.joints_readings.connect_to(read_joint_dispatcher.joints_readings, :type => :buffer, :size => 100)
    log_replay.imu_stim300.orientation_samples_out.connect_to(localization_frontend.orientation_samples, :type => :buffer, :size => 100)
    log_replay.imu_stim300.calibrated_sensors.connect_to(localization_frontend.inertial_samples, :type => :buffer, :size => 100)
    log_replay.gnss_trimble.pose_samples.connect_to(localization_frontend.pose_reference_samples, :type => :buffer, :size => 100)

    log_replay.imu_stim300.calibrated_sensors.connect_to(shark_slam.imu_samples, :type => :buffer, :size => 100)
    log_replay.gnss_trimble.pose_samples.connect_to(shark_slam.gps_pose_samples, :type => :buffer, :size => 100)
    log_replay.imu_stim300.orientation_samples_out.connect_to(shark_slam.orientation_samples, :type => :buffer, :size => 100)

    #############################
    ## TASKS PORTS CONNECTIONS ##
    #############################
    read_joint_dispatcher.joints_samples.connect_to localization_frontend.joints_samples, :type => :buffer, :size => 100
    read_joint_dispatcher.ptu_samples.connect_to ptu_control.ptu_samples, :type => :buffer, :size => 100

    # Start tasks for shark_slam
    shark_slam.start
    read_joint_dispatcher.start
    ptu_control.start
    localization_frontend.start


    # open the log replay widget
    control = Vizkit.control log_replay
    control.speed = 1

    Vizkit.exec
end
