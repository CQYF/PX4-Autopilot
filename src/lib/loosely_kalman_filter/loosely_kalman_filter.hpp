
/**
 * @file statistic_linked_list.hpp
 *
 *
 * @author Cao Yue
 */

#pragma once

#include <stdint.h>
#include <lib/statistic_linked_list/statistic_linked_list.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <drivers/drv_hrt.h>

// 禁止直接访问空闲节点和虚拟节点

using namespace matrix;

template <typename Type, uint8_t HistoryCapacity, uint8_t StateDim>
class LooselyKalmanFilter {
private:
	struct HistoryData {
		uint64_t timestamp;			// 时间戳
		Type z;					// 观测值
		Matrix<Type, 1, StateDim> H;		// 观测矩阵
		Type R;					// 观测噪声
		Vector<Type, StateDim> x;		// 状态
		Matrix<Type, StateDim, StateDim> P;	// 状态协方差

		HistoryData(uint64_t _timestamp, Type _z, Matrix<Type, 1, StateDim> _H, Type _R) :
			timestamp(_timestamp),
			z(_z),
			H(_H),
			R(_R) {
				x.setZero();
				P.setZero();
		}
	};

	StaticLinkedList<HistoryData, HistoryCapacity> history_list_;

	static constexpr uint8_t kStateDim = StateDim;
	static constexpr uint8_t kHistoryCapacity = HistoryCapacity;

	uint64_t lifespan;

	uint8_t need_update_node;
	uint64_t need_update_timestamp;



public:
	LooselyKalmanFilter(std::function<void(Matrix<Type, StateDim, StateDim>&, uint64_t&)> calc_A_,\
		uint64_t lifespan_) :
		calc_A(calc_A_),
		lifespan(lifespan_) {}

	bool insert_data(uint64_t timestamp, Type z, Matrix<Type, 1, StateDim> H, Type R) {

		if(hrt_absolute_time() - timestamp > lifespan) return false;
		if(history_list_.is_full()) return false;

		HistoryData new_data(timestamp, z, H, R);
		uint8_t new_node;
		HistoryData next_data;
		uint8_t next_node;

		// 如果能获取头节点，说明链表中有数据
		if(history_list_.get_head(next_node, next_data)) {
			while(true) {
				// 如果下一个数据的时间戳大于新数据的时间戳，说明还没有找到正确的插入位置，需要继续寻找
				if(next_data.timestamp > new_data.timestamp) {
					if(history_list_.get_next(next_node, next_node, next_data)) {
						continue;
					}
					return false;
				}
				// 找到了插入位置，插入数据
				else {
					history_list_.insert_prev(next_node, new_data, new_node);
					if(need_update_timestamp > timestamp) {
						need_update_timestamp = timestamp;
						need_update_node = new_node;
					}
					return true;
				}
			}
		}
		// 如果链表没有数据，则在头部插入
		else {
			history_list_.push_front(new_data, new_node);
			if(need_update_timestamp > timestamp) {
				need_update_timestamp = timestamp;
				need_update_node = new_node;
			}
			return true;
		}
	}

	void clear_outofdate_data()
	{

	}

	bool update(Vector<Type, StateDim>& x, Matrix<Type, StateDim, StateDim>& P)
	{

	}
};
