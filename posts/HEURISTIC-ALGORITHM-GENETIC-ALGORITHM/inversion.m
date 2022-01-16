function newpop = inversion( pop, pi )%倒位
[ px, py ] = size(pop);
newpop = ones( size(pop));
for i = 1:px
    if( rand < pi )
        newpop(i,1:py/2) = pop(i,py/2+1:py);
        newpop(i,py/2+1:py) = pop(i,1:py/2);%新个体前后半段编码倒位
    else
        newpop(i,:) = pop(i,:);
    end
end
