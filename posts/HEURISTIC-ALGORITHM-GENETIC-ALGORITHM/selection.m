function newpop = selection( pop, fitvalue)
 [ px, py ] = size(pop);
 newpop = zeros( px,py);
 
%概率累计求和
totalfit = sum( fitvalue );
p_fitvalue = fitvalue / totalfit;
p_fitvalue = cumsum( p_fitvalue);  

ms = sort( rand( px,1));  %随机数矩阵，从小到大排列
fitin = 1;  %定义in为进行到的p_value位置
newin = 1;  %定义newin为进行到的ms位置

while newin <= px
    if ms(newin) < p_fitvalue(fitin)
        newpop( newin,:) = pop( fitin,:);
        newin = newin + 1;
    else fitin = fitin + 1;
    end
end

end