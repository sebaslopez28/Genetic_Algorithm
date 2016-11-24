clc
clear all
close all

load x.mat
load y.mat
load mapa.mat

dist=xlsread('dist.xlsx','C9:AC35');

imshow(mapa);hold on;
plot(x,y)
pause(1)

%% 

% Generacion inicial

popSize=200;
xy=[x y];
[N,dims] = size(xy);
n = N;

pop = zeros(popSize,n);
pop(1,:) = (1:n);

for k = 2:popSize
    pop(k,:) = randperm(n);
end

% Evolucion

ite = 500;
% popNew=zeros(1,n,popSize);
 popNew=zeros(popSize,n);

for g = 1:ite
    porc=100*g/ite;
    if mod(porc,10)==0
%         disp(porc)
        disp(sprintf('Progress: %d%c ', porc,'%' ))
    end
    % Costos de la generacion
    b=zeros(popSize,1);
    for i=1:popSize
        for j = 1:n-1    
            %[pop(i,j) pop(i,j+1)];
            b(i) = b(i) + dist(pop(i,j),pop(i,j+1));%+dist(pop(i,1),pop(i,n));
        end
    end
    
    % Encontrando la mejor solucion de la generacion
    [bestb,sol] = min(b);  
    xbest = zeros(size(x));
    ybest = xbest;
    
    for i=1:n
        xbest(i) = x(pop(sol,i)); %sumx(g)=sum(xbest);
        ybest(i) = y(pop(sol,i)); %sumy(g)=sum(ybest);
    end
    
    hold off; imshow(mapa);hold on;
    plot([xbest(:);xbest(1)],[ybest(:);ybest(1)])
    pause(0.01)
    
    pop = pop(randperm(popSize),:);
    
    gen = 0;
    for p=1:5:popSize
%         size(pop)
        gen = gen+1;
        popSub=pop(p:p+4,:);
        d=zeros(size(popSub,1),1);
        for i=1:size(popSub,1)
            for j = 1:n-1 
                d(i) = d(i) + dist(popSub(i,j),popSub(i,j+1));  
            end           
        end    
        [mejor,num] = min(d);
%         popNew(:,:,p)=popSub(num,:)
          popNew(p,:)=popSub(num,:);
    end
    
%      popNew=permute(popNew,[3 2 1]);
     
     % MUTATIONS
     
     % Flip
     for p=2:5:popSize
         k=ceil(n*rand(1));
         popNew(p,:)=[popNew(p-1,1:k-1),flip(popNew(p-1,k:n))];
         %flip(popNew(p-1,:));
     end
     
     % Swap
     for p=3:5:popSize
         k=2;
         for h=1:k+1:n
             popNew(p,h:h+k)=flip(popNew(p-2,h:h+k));
         end
     end
    
    % Shift
     for p=4:5:popSize
         k=floor(n*rand(1));
         popNew(p,:)=circshift(popNew(p-3,:),[0,k]);
     end
     
     % Random    
     for p=5:5:popSize
%          popNew(p,:)=randperm(n);
         popNew(p,:)=flip(popNew(p-4,:));
     end
    
    pop=popNew;
%     popNew=zeros(1,n,popSize);
     popNew=zeros(popSize,n);
    
end