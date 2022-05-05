%% Plot computational statistics from glances
close all;
glances = readmatrix("glances.csv");
glances(end,:) = [];

cpuLoad = glances(:,2);
meanCPU = mean(cpuLoad);

memUtil = glances(:,129);
meanMem = mean(memUtil);

testTime = 600; % 10 minute collect
sampleRate = testTime/(length(glances)-1);

ts = 0:sampleRate:testTime;


% cpu load
figure(1)
plot(ts,cpuLoad,'LineWidth',2);
yline(meanCPU,'--','Color',[0.8500 0.3250 0.0980],'LineWidth',2)
title('CPU Load - 8-Core Intel i7-1165G7 @ 2.80GHz');
ylabel('CPU Load [%]')
xlabel('Time [s]')
legend('Instantaneous CPU Load','Mean CPU Load','Location','southeast')

% memory utilization
figure(2)
plot(ts,memUtil,'Color',[0.6350 0.0780 0.1840],'LineWidth',2);
yline(meanMem,'--','Color',	[0.4940 0.1840 0.5560],'LineWidth',2)
title('Memory Utilization - Intel i7-1165G7 with 16GB RAM');
ylabel('Memory Utilization [%]')
xlabel('Time [s]')
legend('Instantaneous Memory Utilization','Mean Memory Utilization','Location','southeast')