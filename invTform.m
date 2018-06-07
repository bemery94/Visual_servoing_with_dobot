function [inv_transform] = invTform(transform)
    assert(isequal(size(transform),[4,4]));
    
    R = transform(1:3,1:3);
    p = transform(1:3,4);
    
    inv_transform = eye(4,4);
    inv_transform(1:3,1:3) = R';
    inv_transform(1:3,4) = -R' * p;
end