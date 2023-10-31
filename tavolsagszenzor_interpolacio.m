short_dist = [3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 20, 25, 30];
short_voltage = [3, 2.75, 2.35, 2, 1.75, 1.55, 1.4, 1.25, 1.1, 0.9, 0.8, 0.7, 0.65, 0.50, 0.4];
p_short = polyfit(short_voltage,short_dist,5);

long_dist = [15, 20, 30, 40, 50, 60, 70, 80, 90, 130];
long_voltage = [2.75, 2.5, 2, 1.5, 1.25, 1, 0.9, 0.8, 0.75, 0.5];
p_long = polyfit(long_voltage,long_dist,5);
y1 = polyval(p_long,long_voltage);

plot(long_voltage,long_dist,'o')
hold on
plot(long_voltage,y1)
hold off