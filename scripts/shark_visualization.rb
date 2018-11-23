#! /usr/bin/env ruby

require 'orocos'
require 'rock/bundle'
require 'orocos/async'
require 'vizkit'
require 'optparse'

options = {}
options[:hostname] = nil
options[:logfile] = nil

op = OptionParser.new do |opt|
    opt.banner = <<-EOD
    exoter_visualization [options]  </path/to/model/urdf_file>
    EOD
    opt.on '--host=HOSTNAME', String, 'the host we should contact to find RTT tasks' do |host|
        options[:hostname] = host
    end

    opt.on '--log=LOGFILE', String, 'path to the log file' do |log|
        options[:logfile] = log
    end

    opt.on '--help', 'this help message' do
        puts opt
        exit 0
    end
end

args = op.parse(ARGV)
model_file = args.shift

if !model_file
    puts "missing URDF model file argument"
    puts options
    exit 1
end

if options[:hostname]
    Orocos::CORBA.name_service.ip = options[:hostname]
end

# load log files and add the loaded tasks to the Orocos name service
log_replay = Orocos::Log::Replay.open(options[:logfile]) unless options[:logfile].nil?

# If log replay track only needed ports
unless options[:logfile].nil?
    log_replay.track(true)
    log_replay.transformer_broadcaster.rename('foo')
end


Orocos::CORBA::max_message_size = 100000000000
Bundles.initialize
transformation_file = Bundles.find_file('config', 'transforms_scripts_' + 'shark_slam' + '.rb')
puts "Loading transformation file: " + transformation_file
Bundles.transformer.load_conf(transformation_file)

robotVis = Vizkit.default_loader.RobotVisualization
robotVis.modelFile = model_file.dup
robotVis.setPluginName("ExoTer")
Vizkit.vizkit3d_widget.setPluginDataFrame("body", robotVis )

#RigidBodyState of the ground truth
rbsTruth = Vizkit.default_loader.RigidBodyStateVisualization
rbsTruth.setColor(Eigen::Vector3.new(0, 255, 0))#Green rbs
rbsTruth.setPluginName("Reference Pose")
rbsTruth.resetModel(0.4)
Vizkit.vizkit3d_widget.setPluginDataFrame("navigation", rbsTruth)

# Trajectory of the ground truth
truthTrajectory = Vizkit.default_loader.TrajectoryVisualization
truthTrajectory.setColor(Eigen::Vector3.new(0, 1, 0)) #Green line
truthTrajectory.setPluginName("Reference Trajectory")
Vizkit.vizkit3d_widget.setPluginDataFrame("navigation", truthTrajectory)

#RigidBody of the BodyCenter from odometry
odometryRBS = Vizkit.default_loader.RigidBodyStateVisualization
odometryRBS.displayCovariance(true)
odometryRBS.setPluginName("Odometry Pose")
odometryRBS.setColor(Eigen::Vector3.new(255, 0, 0))#Red
odometryRBS.resetModel(0.4)
Vizkit.vizkit3d_widget.setPluginDataFrame("navigation", odometryRBS)

# Odometry robot trajectory
odometryRobotTrajectory = Vizkit.default_loader.TrajectoryVisualization
odometryRobotTrajectory.setColor(Eigen::Vector3.new(1, 0, 0))#Red line
odometryRobotTrajectory.setPluginName("Odometry Trajectory")
Vizkit.vizkit3d_widget.setPluginDataFrame("navigation", odometryRobotTrajectory)


#RigidBody of the BodyCenter from localization
localizationRBS = Vizkit.default_loader.RigidBodyStateVisualization
localizationRBS.displayCovariance(true)
localizationRBS.setPluginName("Robot Pose")
localizationRBS.setColor(Eigen::Vector3.new(255, 255, 255))#White
localizationRBS.resetModel(0.4)
Vizkit.vizkit3d_widget.setPluginDataFrame("navigation", localizationRBS)

# Odometry robot trajectory
localizationRobotTrajectory = Vizkit.default_loader.TrajectoryVisualization
localizationRobotTrajectory.setColor(Eigen::Vector3.new(1, 1, 1))#White line
localizationRobotTrajectory.setPluginName("Robot Trajectory")
Vizkit.vizkit3d_widget.setPluginDataFrame("navigation", localizationRobotTrajectory)

# Point cloud visualizer
pointCloud = Vizkit.default_loader.PointcloudVisualization
pointCloud.setKeepOldData(true)
pointCloud.setMaxOldData(1)
pointCloud.setPluginName("ToF Point Cloud")
Vizkit.vizkit3d_widget.setPluginDataFrame("navigation", pointCloud)

# Point cloud Visual Odometry  visualizer
pointCloudVO = Vizkit.default_loader.PointcloudVisualization
pointCloudVO.setKeepOldData(true)
pointCloudVO.setMaxOldData(1)
pointCloudVO.setPluginName("VO Features")
Vizkit.vizkit3d_widget.setPluginDataFrame("left_camera", pointCloudVO)

#RigidBody of the visual odometry
visualOdometryRBS = Vizkit.default_loader.RigidBodyStateVisualization
visualOdometryRBS.displayCovariance(true)
visualOdometryRBS.setPluginName("Visual Odometry Pose")
visualOdometryRBS.setColor(Eigen::Vector3.new(0, 0, 0))#Black
visualOdometryRBS.resetModel(0.2)
Vizkit.vizkit3d_widget.setPluginDataFrame("navigation", visualOdometryRBS)

# Visual Odometry frame trajectory
visualOdometryTrajectory = Vizkit.default_loader.TrajectoryVisualization
visualOdometryTrajectory.setColor(Eigen::Vector3.new(0, 0, 0))#Black line
visualOdometryTrajectory.setPluginName("Visual Odometry Trajectory")
Vizkit.vizkit3d_widget.setPluginDataFrame("navigation", visualOdometryTrajectory)

#RigidBody of the Iterative Closest Points
icpRBS = Vizkit.default_loader.RigidBodyStateVisualization
icpRBS.displayCovariance(true)
icpRBS.setPluginName("Iterative Closest Points Pose")
icpRBS.setColor(Eigen::Vector3.new(155, 0, 155))
icpRBS.resetModel(0.2)
Vizkit.vizkit3d_widget.setPluginDataFrame("navigation", icpRBS)

# Iterative Closest Points frame trajectory
icpTrajectory = Vizkit.default_loader.TrajectoryVisualization
icpTrajectory.setColor(Eigen::Vector3.new(0.64, 0.64, 0.64))
icpTrajectory.setPluginName("Iterative Closest Points Trajectory")
Vizkit.vizkit3d_widget.setPluginDataFrame("navigation", icpTrajectory)

# Point cloud of features in the localization
featureCloud = Vizkit.default_loader.PointcloudVisualization
featureCloud.setKeepOldData(true)
featureCloud.setMaxOldData(1)
featureCloud.setPluginName("Visual Stereo Features")
Vizkit.vizkit3d_widget.setPluginDataFrame("navigation", featureCloud)

#Contact points FL Wheel (RED)
c0FL = Vizkit.default_loader.RigidBodyStateVisualization
c0FL.displayCovariance(true)
c0FL.setPluginName("FLFoot0")
c0FL.setColor(Eigen::Vector3.new(0, 0, 0))
c0FL.resetModel(0.1)
Vizkit.vizkit3d_widget.setPluginDataFrame("body", c0FL)

#Contact points FR Wheel (GREEN)
c0FR = Vizkit.default_loader.RigidBodyStateVisualization
c0FR.setColor(Eigen::Vector3.new(0, 0, 0))
c0FR.setPluginName("FRFoot0")
c0FR.resetModel(0.1)
c0FR.displayCovariance(true)
Vizkit.vizkit3d_widget.setPluginDataFrame("body", c0FR)

#Contact points ML Wheel (GREEN)
c0ML = Vizkit.default_loader.RigidBodyStateVisualization
c0ML.setColor(Eigen::Vector3.new(0, 0, 0))
c0ML.setPluginName("MLFoot0")
c0ML.resetModel(0.1)
c0ML.displayCovariance(true)
Vizkit.vizkit3d_widget.setPluginDataFrame("body", c0ML)

#Contact points MR Wheel (GREEN)
c0MR = Vizkit.default_loader.RigidBodyStateVisualization
c0MR.setColor(Eigen::Vector3.new(0, 0, 0))
c0MR.setPluginName("MRFoot0")
c0MR.resetModel(0.1)
c0MR.displayCovariance(true)
Vizkit.vizkit3d_widget.setPluginDataFrame("body", c0MR)

#Contact points RL Wheel (BLUE)
c0RL = Vizkit.default_loader.RigidBodyStateVisualization
c0RL.setColor(Eigen::Vector3.new(0, 0, 0))
c0RL.setPluginName("RLFoot0")
c0RL.resetModel(0.1)
c0RL.displayCovariance(true)
Vizkit.vizkit3d_widget.setPluginDataFrame("body", c0RL)

#Contact points RR Wheel (WHITE)
c0RR = Vizkit.default_loader.RigidBodyStateVisualization
c0RR.setColor(Eigen::Vector3.new(0, 0, 0))
c0RR.setPluginName("RRFoot0")
c0RR.resetModel(0.1)
c0RR.displayCovariance(true)
Vizkit.vizkit3d_widget.setPluginDataFrame("body", c0RR)

#Shark slam pose
#Vizkit.vizkit3d_widget.setPluginDataFrame("world", localizationRBS)

# Shark slam  trajectory
#Vizkit.vizkit3d_widget.setPluginDataFrame("world", localizationRobotTrajectory)

# Shark SLAM
shark_slam = Orocos::Async.proxy 'shark_slam'

shark_slam.on_reachable do
        # Shark SLAM Robot pose
        Vizkit.display shark_slam.port('pose_samples_out'), :widget =>localizationRBS

        # Trajectory
        shark_slam.port('pose_samples_out').on_data do |localization_rbs,_|
            localizationRobotTrajectory.updateTrajectory(localization_rbs.position)
        end
end

# Joints Dispatcher for the joints of the robot visualization
read_joint_dispatcher = Orocos::Async.proxy 'read_joint_dispatcher'

read_joint_dispatcher.on_reachable do

    #Joints positions
    read_joint_dispatcher.port('joints_samples').on_data do |joints,_|

        robotVis.updateData(joints)
        #puts "joints #{joints.names}"
    end
end

# Localization Front-End
localization_frontend = Orocos::Async.proxy 'localization_frontend'

localization_frontend.on_reachable do
    #Connect to the ground truth output port (rbs)
    Vizkit.display localization_frontend.port('pose_reference_samples_out'), :widget =>rbsTruth

    #Connect to the ground truth output port (trajectory visualizer)
    localization_frontend.port('pose_reference_samples_out').on_data do |ground_truth,_|
        truthTrajectory.updateTrajectory(ground_truth.position)
    end
end

# JointDispatcher
read_joint_dispatcher = Orocos::Async.proxy 'read_joint_dispatcher'

read_joint_dispatcher.on_reachable do

    #PTU positions
    read_joint_dispatcher.port('ptu_samples').on_data do |ptu,_|

       ptu.names.push("dummy")
       ptu.elements.push(Types::Base::JointState.new(:speed=> 0.00, :position=> 0.00))

       robotVis.updateData(ptu)
       #puts "ptu #{ptu.names}"
    end
end

# Odometry tasks in Asynchronous mode
robot_odometry = Orocos::Async.proxy 'robot_odometry'

robot_odometry.on_reachable do

    #Access to the chains sub_ports
    vector_rbs = robot_odometry.port('fkchains_rbs_out')
    vector_rbs.wait

    Vizkit.display vector_rbs.sub_port([:rbsChain, 0]), :widget => c0FL
    Vizkit.display vector_rbs.sub_port([:rbsChain, 1]), :widget => c0FR
    Vizkit.display vector_rbs.sub_port([:rbsChain, 2]), :widget => c0ML
    Vizkit.display vector_rbs.sub_port([:rbsChain, 3]), :widget => c0MR
    Vizkit.display vector_rbs.sub_port([:rbsChain, 4]), :widget => c0RL
    Vizkit.display vector_rbs.sub_port([:rbsChain, 5]), :widget => c0RR

    # Odometry Robot pose
    Vizkit.display robot_odometry.port('pose_samples_out'), :widget =>odometryRBS

    # Trajectory
    robot_odometry.port('pose_samples_out').on_data do |pose_rbs,_|
        odometryRobotTrajectory.updateTrajectory(pose_rbs.position)
    end
end

# Enable the GUI when the task is reachable
read_joint_dispatcher.on_reachable {Vizkit.vizkit3d_widget.setEnabled(true)} if options[:logfile].nil?
#localization_frontend.on_reachable {Vizkit.vizkit3d_widget.setEnabled(true)} if options[:logfile].nil?

# Disable the GUI until the task is reachable
read_joint_dispatcher.on_unreachable {Vizkit.vizkit3d_widget.setEnabled(false)} if options[:logfile].nil?
#localization_frontend.on_unreachable {Vizkit.vizkit3d_widget.setEnabled(false)} if options[:logfile].nil?

Vizkit.control log_replay unless options[:logfile].nil?
Vizkit.exec


