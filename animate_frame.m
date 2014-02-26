function animate_frame(close_and_save)

persistent first_run tcount tvect rc zc writerObj

if isempty(first_run)
    first_run = 1;
end

if nargin == 1
    if close_and_save
        close(writerObj);
        return;
    end
end

if first_run
    first_run = 0;
    % Intialize plot
    figure(5);
    hold on;
    xc = -25;
    yc = -96;
    t_init = atan2(yc, xc);
    t_step = 0.006; % rad
    t_final = t_init + deg2rad(145); % rad
    tvect = t_init:t_step:t_final;
    tcount = 1;
    zc = 20.5;
    rc = norm([xc yc]);
    set(gca, 'CameraPosition', [xc yc zc]);
    set(gcf, 'color','white');
%     title('Path Planning');
    set(gca,'YTick',[]);
    set(gca,'XTick',[]);
    set(gca,'ZTick',[]);
    axis off
    set(gcf,'Renderer','OpenGL');
%     scrsz = [398 314];
%     set(gcf, 'Position',[1 scrsz(2)/2 scrsz(1)/2 scrsz(2)/2]);
    filename = datestr(now,'mm-dd_HH:MM:SS');
    writerObj = VideoWriter(filename);
    writerObj.FrameRate = 25;
    open(writerObj);
else
    tcount = min(tcount + 1, length(tvect));
    xc = rc * cos(tvect(tcount));
    yc = rc * sin(tvect(tcount));        
    set(gca, 'CameraPosition', [xc yc zc]);
    writeVideo(writerObj,getframe(gcf));
end