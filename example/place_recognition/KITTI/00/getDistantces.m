clear; clc;
addpath(genpath('../../../../matlab/'));

%% Change this to your path
% vel_dir = '/media/gskim/Data/KITTI odo/data_odometry_velodyne/dataset/sequences/00/velodyne/';
vel_dir = '/Users/gsc2001/IIIT-Stuff/RRC/Projects/CollaborativeSLAM/data/kitti/dataset/sequences/00/velodyne/';
% vel_dir = '/Users/gsc2001/IIIT-Stuff/RRC/Projects/CollaborativeSLAM/helper_scripts/kitti_new/dataset/sequences/01/velodyne/';
exp_name = 'kitti_00';

%% Params
% below 3 parameters: same setting as the original paper (G. Kim, 18 IROS)
max_range = 80; % meter 
num_sectors = 60;
num_rings = 20; 

num_candidates = 50; % means Scan Context-50 in the paper.

loop_thres = 0.2; % ### this is a user parameter ###

num_enough_node_diff = 50; 
skip = 1;


%% Main
load('/Users/gsc2001/IIIT-Stuff/RRC/Projects/CollaborativeSLAM/benchmarking/loop_closure/GTposes_kitti_00.mat');

ringkeys = [];
scancontexts = [];

loop_log = [];
num_nodes = length(GTposes);

distances = containers.Map('KeyType','uint32','ValueType', 'any');
disp(num2str(num_nodes))
for ith_node = 1:skip:num_nodes
    disp(num2str(ith_node))
    
    % information 
    idx_query = ith_node;
    ptcloud = KITTIbin2PtcloudWithIndex(vel_dir, idx_query-1);

    sc_query = Ptcloud2ScanContext(ptcloud, num_sectors, num_rings, max_range);
    scancontexts = [scancontexts; sc_query]; % save into database
    
    % ringkey tree
    ringkey = ScanContext2RingKey(sc_query);
    ringkeys = [ringkeys; ringkey];    
end


%% Save the result into csv file 
save('result/kitti_00_scancontexts.mat', "scancontexts");
save('result/kitti_00_ringkeys.mat', "ringkeys");




