
host = 'localhost';
port = 30000;
buffersize = 8196;
tcpipServer = tcpip('localhost', port, 'NetworkRole', 'server');
set(tcpipServer,'OutputBufferSize', buffersize*10);
set(tcpipServer,'InputBufferSize', buffersize);
%set(tcpipServer,'Terminator', char(0));

warning('off','Drake:DisablingSimulinkAutosave')
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
addpath([getenv('APC_BASE'),'/software/jsonlab-1.0/jsonlab']);  % for parsing json
addpath([getenv('APC_BASE'),'/software/json']);                 % for writing json
javaaddpath([getenv('APC_BASE'),'/software/build/share/java/bot2-lcmgl.jar']) % for lcmgl

%% initialize robot here
tic
r = RigidBodyManipulator();
options.urdf_path = [getenv('APC_BASE'), '/catkin_ws/src/apc_config/models/IRB1600ID_drake/irb_1600id.urdf']; 
options.base_offset = [0, 0, 0]';
options.base_rpy = [0, 0, 0]'; 
fprintf('Loading the robot urdf: %s\n', options.urdf_path);
r = addRobotFromURDF(r, options.urdf_path, options.base_offset, options.base_rpy);
toc
%%

while true
    try
        fprintf('Waiting for client at %s:%d...\n', host, 30000);
        flushinput(tcpipServer);
        fopen(tcpipServer);
        tic
        fprintf('Connected\n');
        data_json = fscanf(tcpipServer);
        fprintf('Received data: %s\n', data_json);
        ret_json = ikTrajServer_internal(r, data_json, options);
        %fprintf('Returned data: %s\n', ret_json);
        fprintf(tcpipServer, ret_json);
        toc
        fclose(tcpipServer);
    catch me
        fprintf('Error: %s\n', me.getReport());
        fclose(tcpipServer);
    end
end
