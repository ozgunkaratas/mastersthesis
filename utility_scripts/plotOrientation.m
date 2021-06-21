%% xsens 
xsensquat = transpose([xsens_qw;xsens_qx;xsens_qy;xsens_qz]);
rotm = quat2rotm(xsensquat);
xsens_euler = rotm2eul(rotm(:,:,3500:4000));

tp = theaterPlot('XLimit',[-2 2],'YLimit',[-2 2],'ZLimit',[-2 2]);
op = orientationPlotter(tp,'DisplayName','Fused Data',...
    'LocalAxesLength',2);

%visualize first 1000 samples
for i=1:1000
    plotOrientation(op,inv(-rotm(:,:,i)))
    drawnow
end

tp = theaterPlot('XLimit',[-2 2],'YLimit',[-2 2],'ZLimit',[-2 2]);
op = orientationPlotter(tp,'DisplayName','Fused Data',...
    'LocalAxesLength',2);

%visualize all
for i=1:length(rotm)
    plotOrientation(op, rotm(:,:,i))
    drawnow
end

%% END xsens

%% HL
hlquat= transpose([hl_qw;hl_qx;hl_qy;hl_qz]);
rotm2 = quat2rotm(hlquat);
hl_euler = rotm2eul(rotm2(:,:,3500:4000))

tp = theaterPlot('XLimit',[-2 2],'YLimit',[-2 2],'ZLimit',[-2 2]);
op = orientationPlotter(tp,'DisplayName','Fused Data',...
    'LocalAxesLength',2);

%visualize first 1000 samples
for i=1:1000
    plotOrientation(op,inv(-rotm2(:,:,i)))
    drawnow
end



%visualize all
tp = theaterPlot('XLimit',[-2 2],'YLimit',[-2 2],'ZLimit',[-2 2]);
op = orientationPlotter(tp,'DisplayName','Fused Data',...
    'LocalAxesLength',2);
for i=1:numel(rotm2)
    plotOrientation(op, rotm2(:,:,i))
    drawnow
end

%% END HL




