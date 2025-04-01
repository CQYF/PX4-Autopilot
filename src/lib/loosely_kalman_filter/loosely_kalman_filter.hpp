
/**
 * @file loosely_kalman_filter.hpp
 *
 *
 * @author Cao Yue
 */

#pragma once

#include <stdint.h>
#include <lib/static_linked_list/static_linked_list.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <drivers/drv_hrt.h>

#include <px4_platform_common/log.h>

using namespace matrix;

template <typename Type, uint8_t HistoryCapacity, uint8_t StateDim>
class LooselyKalmanFilter {
private:
	struct HistoryData {
		uint64_t timestamp;			// 时间戳
		Matrix<Type, 1, 1> z;			// 观测值
		Matrix<Type, 1, StateDim> H;		// 观测矩阵
		Matrix<Type, 1, 1> R;			// 观测噪声
		Matrix<Type, StateDim, 1> x;		// 状态
		Matrix<Type, StateDim, StateDim> P;	// 状态协方差

		HistoryData() {}

		HistoryData(uint64_t _timestamp, Matrix<Type, 1, 1> _z,\
			 Matrix<Type, 1, StateDim> _H, Matrix<Type, 1, 1> _R) :
			timestamp(_timestamp),
			z(_z),
			H(_H),
			R(_R) {
				x.setZero();
				P.setZero();
		}
	};

	StaticLinkedList<HistoryData, HistoryCapacity> history_list_;

	uint64_t lifespan;

	bool need_update{false};
	uint8_t need_update_node;
	uint64_t need_update_timestamp;

	using FuncPtr = void(*)(Matrix<Type, StateDim, StateDim>&, uint64_t&);

	FuncPtr calc_F;
	FuncPtr calc_Q;


public:
	LooselyKalmanFilter(FuncPtr calc_F_, FuncPtr calc_Q_, uint64_t lifespan_) :
		lifespan(lifespan_),
		calc_F(calc_F_),
		calc_Q(calc_Q_) {}

	void set_lifespan(uint64_t new_lifespan) {
		lifespan = new_lifespan;
	}

	void info() {
		HistoryData data;
		uint8_t node;
		uint8_t cnt = 0;
		if(history_list_.get_head(node, data))
		{
			do {
				cnt++;
				PX4_INFO("%u %llu", cnt, data.timestamp);
			} while (history_list_.get_next(node, node, data));
		}
	}

	// 插入新数据，新的数据在尾部
	bool insert_data(uint64_t timestamp, Matrix<Type, 1, 1> z, Matrix<Type, 1, StateDim> H, Matrix<Type, 1, 1> R) {
		if(hrt_absolute_time() - timestamp > lifespan) return false;
		if(history_list_.is_full()) return false;

		HistoryData new_data(timestamp, z, H, R);
		uint8_t new_node = 0; // 避免报错
		HistoryData prev_data;
		uint8_t prev_node;

		// 如果能获取尾节点，说明链表中有数据
		if(history_list_.get_tail(prev_node, prev_data)) {
			while(true) {
				// 如果上一个数据的时间戳大于新数据的时间戳，说明还没有找到正确的插入位置，需要继续寻找
				if(prev_data.timestamp > new_data.timestamp) {
					if(history_list_.get_prev(prev_node, prev_node, prev_data)) {
						continue;
					}
					return false;
				}
				// 找到了插入位置，插入数据
				else {
					history_list_.insert_next(prev_node, new_data, new_node);
					if(need_update_timestamp > timestamp) {
						need_update_timestamp = timestamp;
						need_update_node = new_node;
						need_update = true;
					}
					return true;
				}
			}
		}
		// 如果链表没有数据，则在尾部插入
		else {
			history_list_.push_back(new_data, new_node);
			if(need_update_timestamp > timestamp) {
				need_update_timestamp = timestamp;
				need_update_node = new_node;
				need_update = true;
			}
			return true;
		}
	}

	// 清理过期数据
	void clear_outofdate_data()
	{
		HistoryData next_data;
		uint8_t next_node;

		uint64_t now_time = hrt_absolute_time();

		// 如果能获取头节点，说明链表中有数据，可能需要清理
		if(history_list_.get_head(next_node, next_data)) {
			while(true) {
				if(now_time - next_data.timestamp > lifespan) {
					uint8_t tmp_node = next_node;
					if(history_list_.get_next(tmp_node, next_node, next_data)) {
						history_list_.remove(tmp_node);
						continue;
					}
					else
					{
						history_list_.remove(tmp_node);
						break;
					}
				}
				else {
					break;
				}
			}
		}

		// 清理 need_update_timestamp
		need_update_timestamp = now_time;
		need_update = false;
	}

	bool update(Matrix<Type, StateDim, 1>& x_out, Matrix<Type, StateDim, StateDim>& P_out)
	{
		HistoryData unused_data_0;// 防止报错
		HistoryData* data_1 = &unused_data_0;
		uint8_t node_1;
		HistoryData* data_2 = &unused_data_0;
		uint8_t node_2;
		Matrix<Type, StateDim, 1> x;
		Matrix<Type, StateDim, StateDim> P;

		if(need_update) {
			// 如果需要更新的最早节点之前还有节点，则从之前的节点开始，否则从需要更新的最早节点开始
			if( ! history_list_.get_prev(need_update_node, node_1, &data_1)) {
				node_1 = need_update_node;
				history_list_.visit(node_1, &data_1);
			}

			x = data_1->x;
			P = data_1->P;

			while(history_list_.get_next(node_1, node_2, &data_2)) {
				// 计算状态转移矩阵和过程噪声矩阵
				uint64_t dt = data_2->timestamp - data_1->timestamp;
				Matrix<Type, StateDim, StateDim> F;
				Matrix<Type, StateDim, StateDim> Q;
				calc_F(F, dt);
				calc_Q(Q, dt);

				// 预测
				x = F*x;
				P = F*P*F.transpose() + Q;

				Matrix<Type, 1, 1> z = data_2->z;
				Matrix<Type, 1, StateDim> H = data_2->H;
				Matrix<Type, 1, 1> R = data_2->R;

				// 更新
				Matrix<Type, 1, 1> y = z - H*x;
				SquareMatrix<Type, 1> tmp_hph;
				SquareMatrix<Type, 1> tmp_inv;
				tmp_hph = H*P*H.transpose()+R;
				inv(tmp_hph, tmp_inv);
				Matrix<Type, 1, 1> tmp_inv_1(tmp_inv);
				Matrix<Type, StateDim, 1> K = P*H.transpose() * tmp_inv_1;
				x = x + K*y;
				Matrix<Type, StateDim, StateDim> I;
				I.setIdentity();
				P = (I - K*H)*P;

				// 保证P的对称性
				P = (P + P.transpose()) / 2;

				data_2->x = x;
				data_2->P = P;

				node_1 = node_2;
				data_1 = data_2;
			}

			need_update = false;
		}

		// 预测到当前时间
		if(history_list_.get_tail(node_1, &data_1)) {
			x = data_1->x;
			P = data_1->P;
			uint64_t dt = hrt_absolute_time() - data_1->timestamp;
			Matrix<Type, StateDim, StateDim> F;
			Matrix<Type, StateDim, StateDim> Q;
			calc_F(F, dt);
			calc_Q(Q, dt);

			x = F*x;
			P = F*P*F.transpose() + Q;

			x_out = x;
			P_out = P;

			return true;
		}

		return false;
	}
};
