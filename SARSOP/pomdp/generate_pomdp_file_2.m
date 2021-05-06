% Authors: Bernardo Martinez and Jared Strader
% Filename: generate_pomdp_file.m
clear all; close all; clc;
rng(0);

%% define pomdp
% Parameters
discount = 0.95;
values = 'reward';

% States
disp('Computing States.');
% The states are defined by:
%   - (x,y,z) position of the end-effector
%   - (xf,yf,zf) position of the flower
%   - (nx,ny,nz) orientation of the flower
%   - (pollinator) true/false if the pollinator device was actuated
% We assume that the camera is always pointing towards the flower
% For the sake of simplicity, we assume that the flower normal and all
% positions are contained in the x-z plane
vec_x = 0:0.3:1.5;
vec_y = 0;
vec_z = -0.6:0.3:0.6;
vec_xf = 0.9:0.3:1.5;
vec_yf = 0;
vec_zf = -0.3:0.3:0.3;
vec_nx = [0, -sqrt(2),1,-sqrt(2),0];
vec_ny = 0;
vec_nz = [-1, -sqrt(2),0,sqrt(2),1];
vec_pol = 0:1;

idx = 1;
for i=1:length(vec_x)
    for j=1:length(vec_y)
        for k=1:length(vec_z)
            for ii=1:length(vec_xf)
                for jj=1:length(vec_yf)
                    for kk=1:length(vec_zf)
                        for iii=1:length(vec_ny)
                            for jjj=1:length(vec_nz) 
                                for kkk=1:length(vec_pol)
                                    S(idx,1) = vec_x(i);
                                    S(idx,2) = vec_y(j);
                                    S(idx,3) = vec_z(k);
                                    S(idx,4) = vec_xf(ii);
                                    S(idx,5) = vec_yf(jj);
                                    S(idx,6) = vec_zf(kk);
                                    S(idx,7) = vec_ny(iii);
                                    S(idx,8) = vec_nz(jjj);
                                    S(idx,9) = vec_pol(kkk);
                                    idx = idx + 1;
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end

%% Belief
% The belief is defined as [p(s1), p(s2), etc...]
% Here was just define a uniform belief for now given that we know our
% initial end-effector position with high confidence
disp('Computing Initial Belief.');
x = 0;
y = 0;
z = 0;
B = zeros(1,length(S));
N = sum(S(:,1)==0 & S(:,2)==0 & S(:,3)==0 & S(:,9)==0);
B(S(:,1)==0 & S(:,2)==0 & S(:,3)==0 & S(:,9)==0) = 1/N;
B = precision_normalize(B);

%% Transitions
% The transitions are defined as [a, s, s', p(s'|a,s)] for each row
% Here we assume we are able to reach any state given with precision
% given that we know our state
disp('Computing Transitions.');

n_actions = 1+length(vec_x)*length(vec_z);
idx = 1;
action = 1;
for i=1:size(S,1) %end state
    if mod(i,2)==0
        T(idx,1) = action;
        T(idx,2) = i; 
        T(idx,3) = i;
        T(idx,4) = 1.0; 
    else
        T(idx,1) = action;
        T(idx,2) = i; 
        T(idx,3) = i+1;
        T(idx,4) = 1.0; 
    end
    idx = idx + 1;
end
%1:1350
for i=1:length(vec_x)
    for j=1:length(vec_z)
        action = action+1;
        for k=1:size(S,1)
            indices = find(S(:,4)==S(k,4) & S(:,5)==S(k,5) & S(:,6)==S(k,6) & ...
                S(:,7)==S(k,7) & S(:,8)==S(k,8) & S(:,9)==S(k,9) & ...
                S(:,1)==vec_x(i) & S(:,3)==vec_z(j));
            if mod(i,2)==0
                T(idx,1) = action;
                T(idx,2) = k;
                T(idx,3) = k;
                T(idx,4) = 1.0; 
            else
                T(idx,1) = action;
                T(idx,2) = k;
                T(idx,3) = indices;
                T(idx,4) = 1.0; 
            end
            idx = idx + 1;
        end
    end
end
%1351:41850
%% Observations
% The observations are defined as [a, s, p(o=s1|s,a), p(o=s2|s,a), ...]
% We need to determine a better model, but for now out of interest, we
% assume observations are more reliable far from the terminal state
disp('Computing Observations.');
idx = 1;

ay = 1.6;
by = -1.5;
az = 0.06089;
bz = 0.01462;
O = zeros(length(S),length(S)+1);
for i=1:length(S)
    O(i,1) = i;
end

for i=1:size(S,1) %actual state
    if mod(i,2)==0
        O(i,i+1) = 1.0;
    else
        indices = find(S(:,1)==S(i,1) & S(:,3)==S(i,3) & S(:,4)==S(i,4) & S(:,6)==S(i,6) & S(:,9)==S(i,9));
        r = sqrt((S(i,1)-S(i,4))^2+(S(i,3)-S(i,6))^2);
        for j=1:length(indices) %observed state
            P = S(i,1:3) - S(i,4:6);
            N = [0 S(indices(j),7) S(indices(j),8)];
            dp(j) = (dot(P,N)+0.5+(r-0.3)*2)/(1+0.5+(r-0.3)*2);
            if dp(j)<0
                dp(j) = -0.01;
            end
        end
        dp = dp/sum(dp);
        O(i,indices+1) = dp;
    end
    O(i,2:end) = precision_normalize(O(i,2:end)); %normalize
end

% heatmap(O(1:100,2:101));

%     r = sqrt((S(i,1)-S(i,4))^2+(S(i,3)-S(i,6))^2);
%     ph = atan2(S(i,3)-S(i,6), S(i,1)-S(i,4));
%     if ph>1.57
%         ph = ph-3.14;
%     elseif ph<-1.57
%         ph = ph+3.14;
%     end
%     covx = 0.1;
%     covy = ay*r*cos(ph/4) + by*r;
%     covz = az*r*sin(ph*2)^2 + bz*r;

%% Rewards
% The format is [action, start state, end state, reward]
disp('Computing Rewards.');
idx = 1;

r1 = find(((S(:,1)==0.6 & S(:,4)==0.9) | (S(:,1)==0.9 & S(:,4)==1.2) | (S(:,1)==1.2 & S(:,4)==1.5)) & ...
    ((S(:,3)==0.0 & S(:,6)==-0.3) | (S(:,3)==0.3 & S(:,6)==0.0) | (S(:,3)==0.6 & S(:,6)==0.3)) & ...
     S(:,8)==sqrt(2) & S(:,9)==0);

r2 = find(((S(:,1)==0.6 & S(:,4)==0.9) | (S(:,1)==0.9 & S(:,4)==1.2) | (S(:,1)==1.2 & S(:,4)==1.5)) & ...
    ((S(:,3)==0.0 & S(:,6)==0.3) | (S(:,3)==-0.3 & S(:,6)==0.0) | (S(:,3)==-0.6 & S(:,6)==-0.3)) & ...
     S(:,8)==-sqrt(2) & S(:,9)==0);
 
r3 = find(((S(:,1)==0.6 & S(:,4)==0.9) | (S(:,1)==0.9 & S(:,4)==1.2) | (S(:,1)==1.2 & S(:,4)==1.5)) & ...
    ((S(:,3)==0.0 & S(:,6)==0.0) | (S(:,3)==0.3 & S(:,6)==0.3) | (S(:,3)==-0.3 & S(:,6)==-0.3)) & ...
     S(:,8)==0 & S(:,9)==0);
 
r4 = find(((S(:,1)==0.9 & S(:,4)==0.9) | (S(:,1)==1.2 & S(:,4)==1.2) | (S(:,1)==1.5 & S(:,4)==1.5)) & ...
    ((S(:,3)==0.6 & S(:,6)==0.3) | (S(:,3)==0.3 & S(:,6)==0.0) | (S(:,3)==0.0 & S(:,6)==-0.3)) & ...
     S(:,8)==1 & S(:,9)==0);
 
r5 = find(((S(:,1)==0.9 & S(:,4)==0.9) | (S(:,1)==1.2 & S(:,4)==1.2) | (S(:,1)==1.5 & S(:,4)==1.5)) & ...
    ((S(:,3)==-0.6 & S(:,6)==-0.3) | (S(:,3)==-0.3 & S(:,6)==0.0) | (S(:,3)==0.0 & S(:,6)==0.3)) & ...
     S(:,8)==-1 & S(:,9)==0);
 
r = [r1;r2;r3;r4;r5];

idx = 1;
for i=1:2:size(S,1)-1 %action
    if any(i==r)
        R(idx,1) = 1;
        R(idx,2) = i; 
        R(idx,3) = i+1;
        R(idx,4) = +100;
    else
        R(idx,1) = 1;
        R(idx,2) = i; 
        R(idx,3) = i+1;
        R(idx,4) = -100;
    end
    idx = idx + 1;
end

% for i=1351:size(T,1) %action
%     if T(i,2)==T(i,3)
%         R(idx,1) = i; 
%         R(idx,2) = T(i,2); 
%         R(idx,3) = T(i,3);
%         R(idx,4) = -1;
%     else
%         R(idx,1) = i; 
%         R(idx,2) = T(i,2); 
%         R(idx,3) = T(i,3);
%         R(idx,4) = -10;
%     end
%     idx = idx + 1;
% end

for i=2:31 %action
    R(idx,1) = i; 
    R(idx,2) = nan; 
    R(idx,3) = nan;
    R(idx,4) = -10;
    idx = idx + 1;
end

%% write to .pomdp file
%open file
filename = 'manip_2.pomdp';
disp(['Writing to ', filename]);
f_id = fopen(filename,'w');

%write parameters
fprintf(f_id,'# POMDP file for end-effector\n\n');
fprintf(f_id,'discount: %f\n', discount);
fprintf(f_id,'values: %s\n', values);
fprintf(f_id,'states: %u\n', size(S,1));
fprintf(f_id,'actions: %u\n', T(end,1));
fprintf(f_id,'observations: %u\n\n', size(O,1));

%write belief
fprintf(f_id,'start:\n');
for i=1:length(B)
    fprintf(f_id,'%f ', B(i));
end
fprintf(f_id,'\n\n');

%write transitions
fprintf(f_id,'# Transition Probabilities\n');
for i=1:size(T,1)
    fprintf(f_id,'T: %u : %u : %u %f\n', T(i,1)-1,T(i,2)-1,T(i,3)-1,T(i,4));
end
fprintf(f_id,'\n');

%write observations
fprintf(f_id,'# Observation Probabilities\n');
for i=1:size(O,1)
    fprintf(f_id,'O: * : %u\n', i-1);
    for j=2:size(O,2)
        fprintf(f_id,'%1.3f ', O(i,j));
    end
    fprintf(f_id,'\n');
end
fprintf(f_id,'\n');

%write rewards
fprintf(f_id,'# Rewards\n');
for i=1:1350
     fprintf(f_id,'R: %u : %u : %u : * %f\n', R(i,1)-1,R(i,2)-1,R(i,3)-1,R(i,4));
end

% fprintf(f_id,'# Rewards\n');
for i=1351:size(R,1)
     fprintf(f_id,'R: %u : * : * : * %f\n', R(i,1)-1,R(i,4));
end

%close file
fclose(f_id);