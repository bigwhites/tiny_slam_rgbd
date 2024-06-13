#pragma once

#include <DS.h>

AssociateList readAssociationFile(const string &fileName);

// 删除vector中下标为偶数的元素
template <typename T>
void removeIndexElements(vector<T> &vec)
{
    size_t j = 0;
    for (size_t i = 0; i < vec.size(); ++i)
    {
        if (i % 2 != 0)
        { // 仅当索引i是奇数时，才保留该元素
            vec[j] = vec[i];
            ++j;
        }
    }
    vec.resize(j); // 调整向量的大小以删除尾部多余的元素
}