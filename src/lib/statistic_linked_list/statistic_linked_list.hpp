
/**
 * @file statistic_linked_list.hpp
 *
 *
 * @author Cao Yue
 */

#pragma once

// 禁止直接访问空闲节点和虚拟节点

template <typename T, uint8_t DataCapacity>
class StaticLinkedList {
private:
	struct Node {
		uint8_t prev;
		uint8_t next;
	};

	static constexpr uint8_t kNodeCount = DataCapacity + 2;
	static constexpr uint8_t kDataVirtualNode = DataCapacity; // 数据虚拟节点位于数组倒数第二
	static constexpr uint8_t kFreeVirtualNode = DataCapacity + 1; // 空闲虚拟节点位于数组末尾

	// 节点和数据分开存储，下标对齐
	Node nodes_[kNodeCount];
	T data_[DataCapacity];

public:
	StaticLinkedList() {
		// 初始化数据链表虚拟节点，数据链表是一个双向环形链表（但实际上不作为环形链表使用）
		nodes_[kDataVirtualNode].prev = kDataVirtualNode;
		nodes_[kDataVirtualNode].next = kDataVirtualNode;

		// 初始化空闲链表虚拟节点，空闲链表是单向环形链表，
		if (DataCapacity > 0) {
			nodes_[kFreeVirtualNode].prev = kFreeVirtualNode;
			nodes_[kFreeVirtualNode].next = 0;

			for (uint8_t i = 0; i < DataCapacity; ++i) {
				nodes_[i].prev = kFreeVirtualNode; // 所有空闲节点的prev指向虚拟空闲节点
				nodes_[i].next = (i == DataCapacity - 1) ? kFreeVirtualNode : (i + 1);
			}
		} else {
			nodes_[kFreeVirtualNode].prev = kFreeVirtualNode;
			nodes_[kFreeVirtualNode].next = kFreeVirtualNode;
		}
	}

	// 检查是否为数据节点
	bool is_data_node(uint8_t node) const {
		if(node >= kDataVirtualNode) return false;
		return nodes_[node].prev != kFreeVirtualNode;
	}

	// 检查是否为空
	bool is_empty() const {
		return nodes_[kDataVirtualNode].next == kDataVirtualNode;
	}

	// 检查是否为满
	bool is_full() const {
		return nodes_[kFreeVirtualNode].next == kFreeVirtualNode;
	}

	// 访问节点数据
	bool visit(uint8_t node, T& value) const {
		if(is_data_node(node)) {
			value = data_[node];
			return true;
		}
		return false;
	}

	// 获取头节点
	bool get_head(uint8_t& node) const {
		if(!is_empty()) {
			node = nodes_[kDataVirtualNode].next;
			return true;
		}
		return false;
	}
	bool get_head(uint8_t& node, T& value) const {
		if(get_head(node)) {
			value = data_[node];
			return true;
		}
		return false;
	}

	//获取尾节点
	bool get_tail(uint8_t& node) const {
		if(!is_empty()) {
			node = nodes_[kDataVirtualNode].prev;
			return true;
		}
		return false;
	}
	bool get_tail(uint8_t& node, T& value) const {
		if(get_tail(node)) {
			value = data_[node];
			return true;
		}
		return false;
	}

	// 获取下一节点
	bool get_next(uint8_t node, uint8_t& next) const {
		uint8_t next_node;
		if(is_data_node(node)) {
			next_node = nodes_[node].next;
			if(next_node != kDataVirtualNode) {
				next = next_node;
				return true;
			}
		}
		return false;
	}
	bool get_next(uint8_t node, uint8_t& next, T& value) const {
		if(get_next(node, next)) {
			value = data_[next];
			return true;
		}
		return false;
	}

	// 获取上一节点
	bool get_prev(uint8_t node, uint8_t& prev) const {
		uint8_t prev_node;
		if(is_data_node(node)) {
			prev_node = nodes_[node].prev;
			if(prev_node != kDataVirtualNode) {
				prev = prev_node;
				return true;
			}
		}
		return false;
	}
	bool get_prev(uint8_t node, uint8_t& prev, T& value) const {
		if(get_prev(node, prev)) {
			value = data_[prev];
			return true;
		}
		return false;
	}

	// 插入某节点之后
	bool insert_next(uint8_t target, const T& value, uint8_t& new_node) const {
		if(is_data_node(target)) {
			if(PopFreeNode(new_node))
			{
				data_[new_node] = value;
				LinkNext(target, new_node);
				return true;
			}
		}
		return false;
	}

	// 插入某节点之前
	bool insert_prev(uint8_t target, const T& value, uint8_t& new_node) const {
		if(is_data_node(target)) {
			if(PopFreeNode(new_node))
			{
				data_[new_node] = value;
				LinkPrev(target, new_node);
				return true;
			}
		}
		return false;
	}

	// 移除节点
	bool remove(uint8_t node) {
		if(is_data_node(node)) {
			UnlinkNode(node);
			PushFreeNode(node);
			return true;
		}
		return false;
	}

	// 在头部插入
	bool push_front(const T& value, uint8_t& new_node) {
		if (!is_full()) {
			if(PopFreeNode(new_node))
			{
				data_[new_node] = value;
				LinkNext(kDataVirtualNode, new_node);
				return true;
			}
		}
		return false;
	}

	// 在尾部插入
	bool push_back(const T& value, uint8_t& new_node) {
		if (!is_full()) {
			if(PopFreeNode(new_node))
			{
				data_[new_node] = value;
				LinkPrev(kDataVirtualNode, new_node);
				return true;
			}
		}
		return false;
	}

	// 从头部移除
	bool remove_front() {
		if(remove(nodes_[kDataVirtualNode].next)) return true;
		return false;
	}

	// 从尾部移除
	bool remove_back() {
		if(remove(nodes_[kDataVirtualNode].prev)) return true;
		return false;
	}

private:
	void UnlinkNode(uint8_t index) {
		Node& node = nodes_[index];
		nodes_[node.prev].next = node.next;
		nodes_[node.next].prev = node.prev;
	}

	void LinkNext(uint8_t target, uint8_t node) {
		Node& t = nodes_[target];
		const uint8_t old_next = t.next;

		nodes_[node].prev = target;
		nodes_[node].next = old_next;

		t.next = node;
		nodes_[old_next].prev = node;
	}

	void LinkPrev(uint8_t target, uint8_t node) {
		Node& t = nodes_[target];
		const uint8_t old_prev = t.prev;

		nodes_[node].next = target;
		nodes_[node].prev = old_prev;

		t.prev = node;
		nodes_[old_prev].next = node;
	}

	bool PopFreeNode(uint8_t& index) {
		if(is_full()) return false;

		index = nodes_[kFreeVirtualNode].next;
		nodes_[kFreeVirtualNode].next = nodes_[index].next;
		return true;
	}

	void PushFreeNode(uint8_t index) {
		nodes_[index].next = nodes_[kFreeVirtualNode].next;
		nodes_[index].prev = kFreeVirtualNode;
		nodes_[kFreeVirtualNode].next = index;
	}
};
