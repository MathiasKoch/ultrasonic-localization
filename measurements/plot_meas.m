
n = 20;

fid = fopen('30');
[in2, count] = fscanf(fid,'%d,', 2048);
fclose(fid);
data(:,1) = in2(1:n)-300;

fid = fopen('40');
[in2, count] = fscanf(fid,'%d,', 2048);
fclose(fid);
data(:,2) = in2(1:n)-400;

fid = fopen('50');
[in2, count] = fscanf(fid,'%d,', 2048);
fclose(fid);
data(:,3) = in2(1:n)-500;

fid = fopen('60');
[in2, count] = fscanf(fid,'%d,', 2048);
fclose(fid);
data(:,4) = in2(1:n)-600;

fid = fopen('70');
[in2, count] = fscanf(fid,'%d,', 2048);
fclose(fid);
data(:,5) = in2(1:n)-700;

fid = fopen('80');
[in2, count] = fscanf(fid,'%d,', 2048);
fclose(fid);
data(:,6) = in2(1:n)-800;

fid = fopen('90');
[in2, count] = fscanf(fid,'%d,', 2048);
fclose(fid);
data(:,7) = in2(1:n)-900;

fid = fopen('100');
[in2, count] = fscanf(fid,'%d,', 2048);
fclose(fid);
data(:,8) = in2(1:n)-1000;

fid = fopen('110');
[in2, count] = fscanf(fid,'%d,', 2048);
fclose(fid);
data(:,9) = in2(1:n)-1100;

fid = fopen('120');
[in2, count] = fscanf(fid,'%d,', 2048);
fclose(fid);
data(:,10) = in2(1:n)-1200;

fid = fopen('130');
[in2, count] = fscanf(fid,'%d,', 2048);
fclose(fid);
data(:,11) = in2(1:n)-1300;

data = data + 10;
data = data .* 0.1;

figure()
boxplot(data,'Labels',{'30 cm', '40 cm', '50 cm', '60 cm', '70 cm', '80 cm', '90 cm', '100 cm', '110 cm', '120 cm', '130 cm'})
grid on
ylim([-6 6])
ylabel('Error [cm]')
xlabel('Distance')
title('Error in ultrasonic measurements')
tit = get(gca,'Title');
set(tit,'Position',get(tit,'Position') .* [1 1.03 1])

set(tit, 'FontSize', 16)
yh = get(gca,'YLabel'); % Handle of the x label

set(yh,'Position',get(yh,'Position') .* [0.05 1 1])

set(yh, 'FontSize', 14)
set(yh, 'Units', 'Normalized')

xh = get(gca,'XLabel');
set(xh,'Position',get(xh,'Position') .* [1 2 1])
set(xh, 'FontSize', 14)
set(gca,'FontSize',12)



set(gcf,'paperunits','centimeters','Paperposition',[0 0 30 10]);
saveas(gcf,'Measurements.eps','psc2')