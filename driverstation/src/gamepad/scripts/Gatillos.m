clear all
clc
t = 0:.03:3*pi/4;
r = floor(127*t/(3*pi/4));
l = floor(127*sin(2*pi*t));
maxi = 15;
l = [zeros(1,maxi) l(1:end-maxi)];
%l = [zeros(1,maxi) l(1:end-maxi)];
c = 128-r;
%%
clf
plot(t,r,t,l)
hold on 
%plot(t,c,'r')

%%
f = l+c;
plot(t,f,'k')

%% Reconstruir
%mem
clc
R = 0;
L = 0;
Rs = [];
Ls = [];
Ns = [];
m = 128;
for i=1:length(t)
    v = f(i);
    n = v - m;
    
    Ns = [Ns n];
    % si subio(n>0): actualizar L
    if n>0
        Ln = v-128+R;
        Rn = R; % o Rs(end)
%         pause
    elseif n<0
        % si bajo (n<0): actualizar R
        Rn = L+128-v;
        Ln = L;
    else
        Rn = R;
        Ln = L;
    end
    Ls = [Ls Ln];
    Rs = [Rs Rn];
    
    [i v m n R L]
%     pause
    R = Rn;
    L = Ln;
    m = v;
    
end

%% ver

plot(t,Rs,'m', t,Ls,'c')
legend('R', 'L', 'f', 'Rs', 'Ls')