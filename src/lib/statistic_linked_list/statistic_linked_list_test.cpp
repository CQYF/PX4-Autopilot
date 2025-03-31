#include "statistic_linked_list.hpp"
#include <gtest/gtest.h>
#include <vector>

using namespace std;

// 基础功能测试（容量5）
TEST(StaticLinkedListTest, BasicFunctionality) {
    StaticLinkedList<int, 5> list;
    uint8_t node;

    // 测试初始状态
    EXPECT_TRUE(list.is_empty());
    EXPECT_FALSE(list.is_full());
    EXPECT_FALSE(list.get_head(node));
    EXPECT_FALSE(list.get_tail(node));

    // 插入头节点
    ASSERT_TRUE(list.push_front(10, node));
    EXPECT_EQ(node, 0);
    EXPECT_FALSE(list.is_empty());
    EXPECT_FALSE(list.is_full());

    // 验证头节点数据
    int value;
    ASSERT_TRUE(list.get_head(node, value));
    EXPECT_EQ(value, 10);
    EXPECT_EQ(node, 0);

    // 插入尾节点
    ASSERT_TRUE(list.push_back(20, node));
    EXPECT_EQ(node, 1);
    EXPECT_FALSE(list.is_full());

    // 验证尾节点
    ASSERT_TRUE(list.get_tail(node, value));
    EXPECT_EQ(value, 20);
    EXPECT_EQ(node, 1);

    // 在头节点后插入
    uint8_t new_node;
    ASSERT_TRUE(list.insert_next(0, 30, new_node));
    EXPECT_EQ(new_node, 2);

    // 验证中间节点连接
    uint8_t next_node;
    ASSERT_TRUE(list.get_next(0, next_node));
    EXPECT_EQ(next_node, 2);
    ASSERT_TRUE(list.get_prev(next_node, node));
    EXPECT_EQ(node, 0);

    // 删除节点
    ASSERT_TRUE(list.remove(2));
    EXPECT_TRUE(list.is_data_node(0));
    EXPECT_TRUE(list.is_data_node(1));

    // 填满链表
    ASSERT_TRUE(list.push_back(40, node)); // node2
    ASSERT_TRUE(list.push_back(50, node)); // node3
    ASSERT_TRUE(list.push_back(60, node)); // node4
    EXPECT_TRUE(list.is_full());
    EXPECT_FALSE(list.push_back(70, node));

    // 移除尾部
    ASSERT_TRUE(list.remove_back());
    EXPECT_FALSE(list.is_full());

    // 再次插入
    ASSERT_TRUE(list.push_back(70, node));
    EXPECT_TRUE(list.is_full());
}

// 边界条件测试（容量1）
TEST(StaticLinkedListTest, BoundaryConditions) {
    StaticLinkedList<char, 1> list;
    uint8_t node;

    // 测试空链表操作
    EXPECT_FALSE(list.remove_front());
    EXPECT_FALSE(list.remove(0));

    // 插入唯一节点
    ASSERT_TRUE(list.push_back('A', node));
    EXPECT_EQ(node, 0);
    EXPECT_TRUE(list.is_full());
    EXPECT_FALSE(list.push_front('B', node));

    // 删除并验证
    ASSERT_TRUE(list.remove(0));
    EXPECT_TRUE(list.is_empty());
    EXPECT_FALSE(list.is_full());
}

// 节点链接测试（容量3）
TEST(StaticLinkedListTest, NodeLinking) {
    StaticLinkedList<float, 3> list;
    uint8_t n1, n2, n3;

    // 构建链表：头插2个元素，尾插1个元素
    ASSERT_TRUE(list.push_front(1.1f, n1)); // n0
    ASSERT_TRUE(list.push_front(2.2f, n2)); // n1
    ASSERT_TRUE(list.push_back(3.3f, n3));  // n2

    // 验证链表结构：虚拟头 <-> n1 <-> n0 <-> n2 <->虚拟头
    uint8_t current;
    ASSERT_TRUE(list.get_head(current));
    EXPECT_EQ(current, n2); // 最后push_front的是n2（即n1）

    vector<float> expected = {2.2f, 1.1f, 3.3f};
    vector<float> actual;
    uint8_t node;
    bool success = list.get_head(node);
    while(success) {
        float val;
        list.visit(node, val);
        actual.push_back(val);
        success = list.get_next(node, node);
    }
    EXPECT_EQ(actual, expected);
}

// 异常操作测试（容量2）
TEST(StaticLinkedListTest, InvalidOperations) {
    StaticLinkedList<string, 2> list;
    uint8_t node = 5; // 无效节点

    // 访问无效节点
    string val;
    EXPECT_FALSE(list.visit(3, val));  // 超出范围
    EXPECT_FALSE(list.visit(2, val));  // 虚拟节点

    // 在无效节点插入
    EXPECT_FALSE(list.insert_next(2, "test", node)); // 虚拟节点
    EXPECT_FALSE(list.insert_prev(5, "test", node)); // 不存在节点

    // 操作空闲节点
    list.push_back("first", node); // node0
    uint8_t free_node = 1;
    EXPECT_FALSE(list.remove(free_node)); // 空闲节点不可删除
}

// 容量为0的特殊测试
TEST(StaticLinkedListTest, ZeroCapacity) {
    StaticLinkedList<double, 0> list;

    EXPECT_TRUE(list.is_empty());
    EXPECT_TRUE(list.is_full());
    uint8_t node;
    EXPECT_FALSE(list.push_front(1.0, node));
    EXPECT_FALSE(list.remove_front());
}

// 混合操作测试（容量4）
TEST(StaticLinkedListTest, MixedOperations) {
    StaticLinkedList<int, 4> list;
    uint8_t nodes[4];

    // 填充数据
    for(int i=0; i<4; ++i) {
        ASSERT_TRUE(list.push_back(i*10, nodes[i]));
    }

    // 验证满状态
    EXPECT_TRUE(list.is_full());
    uint8_t dummy;
    EXPECT_FALSE(list.push_front(100, dummy));

    // 删除中间两个节点
    ASSERT_TRUE(list.remove(nodes[1]));
    ASSERT_TRUE(list.remove(nodes[2]));

    // 验证剩余节点
    vector<int> values;
    uint8_t current;
    if(list.get_head(current)) {
        do {
            int val;
            list.visit(current, val);
            values.push_back(val);
        } while(list.get_next(current, current));
    }
    ASSERT_EQ(values, vector<int>({0, 30}));

    // 再次插入
    ASSERT_TRUE(list.insert_prev(nodes[3], 25, dummy)); // 在30前插入
    ASSERT_TRUE(list.insert_next(nodes[0], 5, dummy));  // 在0后插入

    // 验证最终结构
    values.clear();
    if(list.get_head(current)) {
        do {
            int val;
            list.visit(current, val);
            values.push_back(val);
        } while(list.get_next(current, current));
    }
    EXPECT_EQ(values, vector<int>({0,5,25,30}));
}
