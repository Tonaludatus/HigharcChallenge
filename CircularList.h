#pragma once

#include <cassert>

template <typename Data>
struct CListItem {
	Data data;
	CListItem* next;
	CListItem* prev;
	explicit CListItem(Data&& d) : data(std::move(d)), next(this), prev(this) {}
	template <typename... Args>
	CListItem(Args&&... args) : data(std::forward<Args>(args)...), next(this), prev(this) {}
};

template <typename Data>
class CList {
private:
	CListItem<Data>* head = nullptr;
public:
	class iterator {
	private:
		friend class CList;
		const CList* list; // Must be a pointer for op= to work
		CListItem<Data>* item;
		int revolution_num = 0;
		iterator(const CList& l, CListItem<Data>* i, int r) : list(&l), item(i), revolution_num(r) {}

	public:
		 iterator& operator++() {
			if (item->next == list->head) {
				++revolution_num;
			}
			item = item->next;
			return *this;
		}
		iterator& operator--() {
			if (item->prev == list->head) {
				--revolution_num;
			}
			item = item->prev;
			return *this;
		}
		iterator circularAdvancedBy(int n) const {
			iterator ret{ *this };
			if (n >= 0) {
				for (int i = 0; i < n; ++i) {
					ret.item = ret.item->next;
				}
			}
			else {
				for (int i = 0; i > n; --i) {
					ret.item = ret.item->prev;
				}
			}
			return ret;
		}
		const Data& operator*() const { return item->data; }
		Data& operator*() { return item->data; }
		const Data* operator->() const { return &(item->data); }
		Data* operator->() { return &(item->data); }
		bool operator==(const iterator& other) const {
			return list == other.list && item == other.item && (revolution_num == other.revolution_num || item == nullptr);
		}
		bool operator!=(const iterator& other) const {
			return !operator==(other);
		}
		iterator& operator=(const iterator& other) {
			item = other.item;
			revolution_num = other.revolution_num;
			return *this;
		}
	};
private:
	iterator insertImpl(const iterator& before_this, CListItem<Data>* item) {
		if (before_this.item == nullptr) {
			head = item;
			return iterator(*this, head, 0);
		}
		if (before_this == begin()) {
			head = item;
		}
		item->prev = before_this.item->prev;
		item->next = before_this.item;
		before_this.item->prev = item;
		item->prev->next = item;
		return iterator(*this, item, 0);
	}

	iterator pushFrontImpl(CListItem<Data>* item) {
		auto ret = insertImpl(begin(), item);
		head = item;
		return ret;
	}

	iterator pushBackImpl(CListItem<Data>* item) {
		auto ret = insertImpl(end(), item);
		return ret;
	}
public:
	~CList() {
		auto it = head;
		if (head != nullptr) {
			head->prev->next = nullptr;
		}
		while (it != nullptr) {
			auto to_del = it;
			it = it->next;
			delete to_del;
		}
	}
	const iterator end() const {
		return iterator(*this, head, 1);
	}
	const iterator begin() const {
		return iterator(*this, head, 0);
	}
	const iterator rend() const {
		return iterator(*this, head, -1);
	}
	const iterator rbegin() const {
		return iterator(*this, head, 0);
	}

	template <typename... Args>
	iterator emplace(const iterator& before_this, Args&&... to_insert) {
		return insertImpl(before_this, new CListItem<Data>(std::forward<Args>(to_insert)...));
	}

	iterator insert(const iterator& before_this, Data&& to_insert) {
		return insertImpl(before_this, new CListItem<Data>(std::move(to_insert)));
	}

	iterator push_front(Data&& to_insert) {
		return pushFrontImpl(new CListItem<Data>(std::move(to_insert)));
	}

	template <typename... Args>
	iterator emplace_front(Args&&... to_insert) {
		return pushFrontImpl(new CListItem<Data>(std::forward<Args>(to_insert)...));
	}

	iterator push_back(Data&& to_insert) {
		return pushBackImpl(new CListItem<Data>(std::move(to_insert)));
	}

	template <typename... Args>
	iterator emplace_back(Args&&... to_insert) {
		return pushBackImpl(new CListItem<Data>(std::forward<Args>(to_insert)...));
	}
};