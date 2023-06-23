close all;

Array=readtable('test_results/field_test/field_A_crd5_sb125_sf11_50.csv');

t = datetime(table2array(Array(:,1)),'InputFormat','yyyy-MM-dd HH:mm:ss.SSSSSS');
delay = table2array(Array(:, 2));
msgID = table2array(Array(:, 3));
nodeID = table2array(Array(:, 4));
rssi = table2array(Array(:, 5));
snr = table2array(Array(:, 6));
battery = table2array(Array(:, 7));
dir = table2array(Array(:, 8));

n = size(msgID);
n = n(1);

a=0;b=0;c=0;d=0;e=0;f=0;g=0;h=0;ii=0;

time = t-t(1);

ul_packets = 0;
dl_packets = 0;

% Plot Settings
markerSize = 15;
lineThickness = 0.0567;

indxs(1,3)=0;


%% Claculate Average RSSI and SNR and AvgDelay
prevID = -1;

for i=1:n
    if(dir(i) == 0)
        if(nodeID(i) == 1)
            a = a+1;
            indxs(a,1) = i;
        end
        if (nodeID(i) == 2)
            b = b+1;
            indxs(b,2) = i; 
        end
        if (nodeID(i) == 3)
            c = c+1;
            indxs(c,3) = i;
        end
        ul_packets = ul_packets + 1;
    end
    if(dir(i) == 1)
        
        if(nodeID(i) == 1)
            if(nodeID(i) ~= prevID)
                if(any(msgID(indxs(1:a,1)) == msgID(i)))
                    d = d+1;
                end
            end
            g = g+1;
            if(any(msgID(indxs(1:a,1)) == msgID(i)))
                indxs_dl(d,1) = i;
            end
        end
        if (nodeID(i) == 2)
            if(nodeID(i) ~= prevID)
                if(any(msgID(indxs(1:b,2)) == msgID(i)))
                    e = e+1;
                end
            end
            h = h+1;
            if(any(msgID(indxs(1:b,2)) == msgID(i)))
                indxs_dl(e,2) = i; 
            end
        end
        if (nodeID(i) == 3)
            if(nodeID(i) ~= prevID)
                if(any(msgID(indxs(1:c,3)) == msgID(i)))
                    f = f+1;
                end
            end
            ii = ii+1;
            if(any(msgID(indxs(1:c,3)) == msgID(i)))
                indxs_dl(f,3) = i;
            end
        end
        dl_packets = dl_packets + 1;
        prevID = nodeID(i);
    end
end

avg_rssi(1) = mean(rssi(indxs(1:a,1)));
avg_rssi(2) = mean(rssi(indxs(1:b,2)));
%avg_rssi(3) = mean(rssi(indxs(1:c,3)));

avg_snr(1) = mean(snr(indxs(1:a,1)));
avg_snr(2) = mean(snr(indxs(1:b,2)));
%avg_snr(3) = mean(snr(indxs(1:c,3)));

%% Calculate Average Delay Time

avg_delay(1) = mean(delay(indxs(1:a,1))-delay(indxs_dl(1:d,1)));
avg_delay(2) = mean(delay(indxs(1:b,2))-delay(indxs_dl(1:e,2)));
%avg_delay(3) = mean(delay(indxs(1:c,3))-delay(indxs_dl(1:f,3)));
avg_delay_m = mean(avg_delay);

mID = max(msgID);

avg_delay_m

%% Calculate Package Loss

packet_loss = (1 - ul_packets/dl_packets)*100;
packet_loss

node1_pl = (1- a/g)*100;
node2_pl = (1- b/h)*100;
%node3_pl = (1- c/ii)*100;

%% Plot RSSI vs Time
figure();
hold on

line([min(time),max(time)],[avg_rssi(1),avg_rssi(1)], 'Color', [0 0.4470 0.7410], 'LineWidth',lineThickness);
line([min(time),max(time)],[avg_rssi(2),avg_rssi(2)], 'Color', [0.8500 0.3250 0.0980], 'LineWidth',lineThickness);
%line([min(time),max(time)],[avg_rssi(3),avg_rssi(3)], 'Color', [0.9290 0.6940 0.1250], 'LineWidth',lineThickness);

scatter(time(indxs(1:a,1)), rssi(indxs(1:a,1)),markerSize,'MarkerEdgeColor',[0 0.3470 0.6410],...
                                                          'MarkerFaceColor',[0 0.4470 0.7410],...
                                                          'LineWidth',1);
                              
scatter(time(indxs(1:b,2)), rssi(indxs(1:b,2)),markerSize,'MarkerEdgeColor',[0.7500 0.2250 0.0680],...
                                                          'MarkerFaceColor',[0.8500 0.3250 0.0980],...
                                                          'LineWidth',1);
                                
%scatter(time(indxs(1:c,3)), rssi(indxs(1:c,3)),markerSize,'MarkerEdgeColor',[0.8290 0.5940 0.0250],...
                                                          %'MarkerFaceColor',[0.9290 0.6940 0.1250],...
                                                          %'LineWidth',1);
                                        
grid on;
ylim([-120 -30]);
xlim([time(1) time(a+b+c)]);
xtickformat('mm:ss');
legend('Node 1 Average', 'Node 2 Average', 'Node 3 Average', 'Node 1 Packets', 'Node 2 Packets', 'Node 3 Packets','Location','best');
xlabel("Time (mm:ss)");
ylabel("RSSI (dB)");


%% Plot SNR vs Time
figure();
hold on

line([min(time),max(time)],[avg_snr(1),avg_snr(1)], 'Color', [0 0.4470 0.7410]);
line([min(time),max(time)],[avg_snr(2),avg_snr(2)], 'Color', [0.8500 0.3250 0.0980]);
%line([min(time),max(time)],[avg_snr(3),avg_snr(3)], 'Color', [0.9290 0.6940 0.1250]);

scatter(time(indxs(1:a,1)), snr(indxs(1:a,1)),markerSize,'MarkerEdgeColor',[0 0.3470 0.6410],...
                                                         'MarkerFaceColor',[0 0.4470 0.7410],...
                                                         'LineWidth',1);
                              
scatter(time(indxs(1:b,2)), snr(indxs(1:b,2)),markerSize,'MarkerEdgeColor',[0.7500 0.2250 0.0680],...
                                                         'MarkerFaceColor',[0.8500 0.3250 0.0980],...
                                                         'LineWidth',1);
                                
%scatter(time(indxs(1:c,3)), snr(indxs(1:c,3)),markerSize,'MarkerEdgeColor',[0.8290 0.5940 0.0250],...
                                                         %'MarkerFaceColor',[0.9290 0.6940 0.1250],...
                                                         %'LineWidth',1);
                                        
grid on;
ylim([-5 13]);
xlim([time(1) time(a+b+c)]);
xtickformat('mm:ss');
legend('Node 1 Average', 'Node 2 Average', 'Node 3 Average', 'Node 1 Packets', 'Node 2 Packets', 'Node 3 Packets','Location','best');
xlabel("Time (mm:ss)");
ylabel("SNR (dB)");




%% Plot Delays

figure();
hold on

line([min(time),max(time)],[avg_delay(1),avg_delay(1)], 'Color', [0 0.4470 0.7410]);
line([min(time),max(time)],[avg_delay(2),avg_delay(2)], 'Color', [0.8500 0.3250 0.0980]);
%line([min(time),max(time)],[avg_delay(3),avg_delay(3)], 'Color', [0.9290 0.6940 0.1250]);

scatter(time(indxs(1:a,1)), delay(indxs(1:a,1))-delay(indxs_dl(1:d,1)),markerSize,'MarkerEdgeColor',[0 0.3470 0.6410],...
                                                                                  'MarkerFaceColor',[0 0.4470 0.7410],...
                                                                                  'LineWidth',1);
                              
scatter(time(indxs(1:b,2)), delay(indxs(1:b,2))-delay(indxs_dl(1:e,2)),markerSize,'MarkerEdgeColor',[0.7500 0.2250 0.0680],...
                                                                                  'MarkerFaceColor',[0.8500 0.3250 0.0980],...
                                                                                  'LineWidth',1);
                                
%scatter(time(indxs(1:c,3)), delay(indxs(1:c,3))-delay(indxs_dl(1:f,3)),markerSize,'MarkerEdgeColor',[0.8290 0.5940 0.0250],...
                                                                                  %'MarkerFaceColor',[0.9290 0.6940 0.1250],...
                                                                                  %'LineWidth',1);
                                        
grid on;
ylim([150 210]);
xlim([time(1) time(a+b+c)]);
xtickformat('mm:ss');
legend('Node 1 Average', 'Node 2 Average', 'Node 3 Average', 'Node 1', 'Node 2', 'Node 3','Location','best');
xlabel("Time (mm:ss)");
ylabel("Packet Delay (ms)");





