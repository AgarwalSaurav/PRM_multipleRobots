dataPRM = zeros(10, 5);
dataPRM(10, :) = [271.42, 1.60, 1, 302, 29];
dataPRM(9, :) = [242, 1.43, 1, 249, 30];
dataPRM(8, :) = [206, 0.99, 0, 241, 37];
dataPRM(7, :) = [157, 5.61, 0, 248, 33];
dataPRM(6, :) = [118, 4.7, 0, 217, 45];
dataPRM(5, :) = [111, 3.7, 0, 159, 31];
dataPRM(4, :) = [104, 1.59, 0, 121, 12];
dataPRM(3, :) = [97.9, 1.6, 0, 49, 9];
dataPRM(2, :) = [91.9, 0.61, 0, 49, 9];
dataPRM(1, :) = [75.5, 0.27, 0, 15, 12];



plot(dataPRM(:, 1), 'Marker', 'o', 'linewidth', 2)
set(0,'DefaultTextInterpreter', 'latex')
%axis square
ax = gca;
ax.TickLabelInterpreter = 'latex'; 
title('{\bf Learning time with number of robots}')
xlabel('Number of robots')
ylabel('Learning time (s)')
set(ax,'fontsize',20)
grid on
%trimFigure(ax);
%saveas(gcf, lT_plot.png)
%saveas(gcf,lT_plot.eps)


figure
plot(dataPRM(:, 2), 'Marker', 'o', 'linewidth', 2)
set(0,'DefaultTextInterpreter', 'latex')
%axis square
ax = gca;
ax.TickLabelInterpreter = 'latex'; 
title('{\bf Average query (100 queries) time with number of robots}')
xlabel('Number of robots')
ylabel('Query time (s)')
set(ax,'fontsize',20)
grid on

figure
plot(dataPRM(:, 4), 'Marker', 'o', 'linewidth', 2)
set(0,'DefaultTextInterpreter', 'latex')
%axis square
ax = gca;
ax.TickLabelInterpreter = 'latex'; 
title('{\bf Number of connected components with number of robots}')
xlabel('Number of robots')
ylabel('Number of connected componentse')
set(ax,'fontsize',20)
grid on

figure
plot(dataPRM(:, 5), 'Marker', 'o', 'linewidth', 2)
set(0,'DefaultTextInterpreter', 'latex')
%axis square
ax = gca;
ax.TickLabelInterpreter = 'latex'; 
title('{\bf Number of unconnected nodes with number of robots}')
xlabel('Number of robots')
ylabel('Number of unconnected nodes')
set(ax,'fontsize',20)
grid on

