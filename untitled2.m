% set choice as 0 for the result of task 2C
% set choice as 1-2-3-4-5-6 for the configurations of the tasks 3B and 3C 
choice = 0;

if choice == 0 
    bag = ros2bagreader('C:\Users\bambr\Desktop\rosbag2_sii\rosbag2_2024_12_14-11_58_40_0.db3')
elseif choice == 1
    bag = ros2bagreader('C:\Users\bambr\Desktop\rosbag2_config_init\rosbag2_2024_12_14-15_53_40_0.db3')
elseif choice == 2
    bag = ros2bagreader('C:\Users\bambr\Desktop\rosbag2_config1\rosbag2_2024_12_14-15_40_45_0.db3')
elseif choice == 3
    bag = ros2bagreader('C:\Users\bambr\Desktop\rosbag2_config2\rosbag2_2024_12_14-16_14_39_0.db3')
elseif choice == 4
    bag = ros2bagreader('C:\Users\bambr\Desktop\rosbag2_config3\rosbag2_2024_12_14-16_35_25_0.db3')
elseif choice == 5
    bag = ros2bagreader('C:\Users\bambr\Desktop\rosbag2_config4\rosbag2_2024_12_14-17_01_06_0.db3')
elseif choice == 6
    bag = ros2bagreader('C:\Users\bambr\Desktop\rosbag2_punto4\rosbag2_2024_12_18-14_52_02_0.db3')

end
%%
topicList = bag.AvailableTopics;
disp(topicList);

msgs = readMessages(select(bag, 'Topic', '/global_costmap/published_footprint'));
%%
% Numero di messaggi letti
n = numel(msgs);

% Pre-allocazione per i dati
XValues = zeros(n,1);  
YValues = zeros(n,1);

for i = 1:n
    XValues(i) = msgs{i,1}.polygon.points.y;
    YValues(i) = msgs{i,1}.polygon.points.x;
end
%%
OffsetX=zeros(n,1);
OffsetX(:,1)=3;
OffsetY=zeros(n,1);
OffsetY(:,1)=3.5;
XValues(:,1)=XValues(:,1)-OffsetX(:,1);
YValues(:,1)=OffsetY(:,1)-YValues(:,1);
%%
if choice == 0
    XPUNTI = [6.5,-1.6,6,0];
    YPUNTI = [-1.4,-2.5,4,3];
    didascalie = {'GOAL 3', 'GOAL 4', 'GOAL 2', ' GOAL 1'}; % Testo per ogni punto
elseif choice == 6
    XPUNTI = [-3.78,-3];
    YPUNTI = [0.18,3.5];
    didascalie = {'GOAL 1-2', 'GOAL 3'}; % Testo per ogni punto    
else
    XPUNTI = [-9.15,-6.93,-3.58,3.9,3.41,9.43,9.43,1.46]
    YPUNTI = [4.3,-3.69,0.06,0.14,4.10,4.31,-4.61,-4.25]
    didascalie = {'GOAL 1', 'GOAL 2', 'GOAL 3', ' GOAL 4', 'GOAL 5', 'GOAL 6', 'GOAL 7', 'GOAL 8'}; % Testo per ogni punto
end

plot(XValues, YValues)
xlabel('X');
ylabel('Y');
title('Traiettoria del robot')
grid on;
hold on
plot(XPUNTI,YPUNTI,'ro', 'MarkerSize', 5, 'LineWidth', 2);

for i = 1:length(XPUNTI)
    text(XPUNTI(i), YPUNTI(i) + 0.3, didascalie{i}, 'FontSize', 10, 'Color', 'b'); % Offset in y per visibilità
end
hold off
