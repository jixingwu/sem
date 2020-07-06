# C++11语法学习
## std::thread
[cerebro_node.cpp]499:
- 开辟线程```std::thread dot_product_th(&Cerebro::run, &cer);```
```cer```作为参数传入到```Cerebro::run()```中运行
- ```dot_product_th.join();//等待dot_product_th线程运行完```

## std::atomic
[Cerebro.h]
一个模板类，从不同线程访问不会导致数据竞争

## std::map< ros::Time, DataNode* > t__DataNode
有时间戳的DataNode类
用t__DataNode.at()访问某个元素

## std::rebing()
反向迭代器