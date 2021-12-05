% Display learnt model
figure('PaperSize',[20.98404194812 29.67743169791],...
    'Color',[1 1 1]);
%plot(X, y, 'k+', 'MarkerSize', 17)
%plot(t,x,'k-','LineWidth',2) % Plot both signals.
hold on
f  = [mf+2*(sf),flipdim(mf-2*(sf),2)]';
h(1) = fill([xstar, flipdim(xstar,2)], f, [6 6 6]/8, 'EdgeColor', [6 6 6]/8);
hold on
h(2) = plot(xstar,mf,'b-');
h(3) = plot(t,x,'k-','LineWidth',2); % Plot both signals.
%h(3) = plot(X, y, 'k+', 'MarkerSize', 17);
%h(4) = scatter(X(1:60), y(1:60), 'r','filled');
h(4) = scatter(X(1:60), y(1:60), 'r','filled');

legend(h,'Predictive Standard Deviation','Predictive Mean','GT','Training Points')
set(legend,...
    'Position',[0.434313057085632 0.773798076923083 0.150347222222222 0.143990384615385]);
xlabel('time (in seconds)');
ylim(gca,[-4 3]);
