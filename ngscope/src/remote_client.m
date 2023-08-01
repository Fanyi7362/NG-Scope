% matlab as remote client (socket receiver)
% receive ue_dci from remote server
% generated from dci_sink_client_thread.c

% clear all; close all; clc;

% parameters
server_IP = '127.0.0.1';
server_port = 6767;

% create a UDP object
u = udp(server_IP, 'LocalPort', server_port);

% set the InputBufferSize to your desired size
u.InputBufferSize = 1400;

% open the connection
fopen(u);

% keep receiving data
while true
    if u.BytesAvailable > 0
        data = fread(u, u.BytesAvailable);
        disp('Received data: ')
        disp(data)
    else
        pause(0.1); % Avoid tight loop when no data
    end
end

% remember to close and delete the UDP object when finished
fclose(u);
delete(u);
