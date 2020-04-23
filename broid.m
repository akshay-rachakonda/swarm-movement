tic
clc
clear all
global noparticle range
noparticle=10;% number of particles
range=1; % sensing range for particle
sensingrange= range*ones(1,noparticle);
maxstep=300;% no of iterations
wait=0.8;

%space initialization
uplimit_1=100;
dwlimit_1=-100;
assert(dwlimit_1<uplimit_1)

uplimit_2=100;
dwlimit_2=-100;
assert(dwlimit_2<uplimit_2)

uplimit_3=100;
dwlimit_3=-100;
assert(dwlimit_3<uplimit_3)

%virtual limit for avoiding collision with boundaries
dwspace= repmat([dwlimit_1;dwlimit_2;dwlimit_3],1,noparticle);
upspace= repmat([uplimit_1;uplimit_2;uplimit_3],1,noparticle);

% virtual limit for avoiding collision with boundary, concern dimension of
% particle
 dw= dwspace+range;
 up=upspace+range;
 global spacesize
 spacesize= upspace-dwspace;
% swarm
 swarm= dw+ spacesize/2 + rand(3,noparticle).*spacesize/100; %initial position
 velscale=0.01; %ratio of max velocity to space size
velmax=velscale*spacesize; % 0.01*200
velo=rand*velmax; % initial velocity 2*0.6

swarmx=swarm(1,:); % position x
swarmy=swarm(2,:); % position y
swarmz=swarm(3,:); % position z
M=moviein(1/10000);
for step=1:maxstep
    M(:,step) = getframe;
    velo= updateboidvelocity(swarm,velo)+rand*velmax/10;
    % check speed limit
    for idim= 1:3
        for perno= 1:noparticle 
            if velo(idim,perno)>velmax(idim,perno)
                velo(idim,perno)=velmax(idim,perno);
            end
        end
    end
    % update position
    tempos= swarm.*velo;
    % check space limit
     for idim= 1:3
        for perno= 1:noparticle 
            if tempos(idim,perno)>up(idim,perno)
             tempos(idim,perno)=rand*(dw(idim,perno))+spacesize(idim,perno);
            elseif tempos(idim,perno)<dw(idim,perno)  
             tempos(idim,perno)=rand*(dw(idim,perno))+spacesize(idim,perno);
            end
        end
    end
    swarm= tempos;
%   graphic
swarmx=[swarmx;swarm(1,:)];
swarmy=[swarmy;swarm(2,:)];
swarmz=[swarmz;swarm(3,:)];
% motion track
clf;
scatter3(0,0,0,'h');
xlabel('x coordinate','FontSize',11);
ylabel('y cordinate','FontSize',11);
zlabel('z cordinate','FontSize',11);
str= ['Step', num2str(step)];
title(str,'FontSize',20);
    hold on
    scatter3(swarmx(step,:),swarmy(step,:),swarmz(step,:),'r+')
    hold on
    swarmstep=[swarmx(step,:);swarmy(step,:);swarmz(step,:)]';
   % viscircles(swarmstep,sensingrange,'EdgeColor','b');
    axis([-100 300 -100 30*10 -100 300]);
    hold on
    pause(wait/2);
end
pause(wait/10);
close all
toc