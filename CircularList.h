#pragma once

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
		const CList& list;
		CListItem<Data>* item;
		int revolution_num = 0;
		iterator(const CList& l, CListItem<Data>* i, int r) : list(l), item(i), revolution_num(r) {}
	public:
		 iterator& operator++() {
			if (item->next == list.head) {
				++revolution_num;
			}
			item = item->next;
			return *this;
		}
		iterator& operator--() {
			if (item->prev == list.head) {
				--revolution_num;
			}
			item = item->prev;
			return *this;
		}
		const Data& operator*() const { return item->data; }
		Data& operator*() { return item->data; }
		const Data* operator->() const { return &(item->data); }
		Data* operator->() { return &(item->data); }
		bool operator==(const iterator& other) const {
			return &list == &other.list && item == other.item && (revolution_num == other.revolution_num || item == nullptr);
		}
		bool operator!=(const iterator& other) const {
			return !operator==(other);
		}
	};

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
		if (before_this.item == nullptr) {
			head = new CListItem<Data>(std::forward<Args>(to_insert)...);
			return iterator(*this, head, 0);
		}
		auto item = new CListItem<Data>(std::forward<Args>(to_insert)...);
		item->prev = before_this.item->prev;
		item->next = before_this.item;
		before_this.item->prev = item;
		item->prev->next = item;
		return iterator(*this, item, 0);
	}

	iterator insert(const iterator& before_this, Data&& to_insert) {
		if (before_this.item == nullptr) {
			head = new CListItem<Data>(std::move(to_insert));
			return iterator(*this, head, 0);
		}
		auto item = new CListItem<Data>(std::move(to_insert));
		item->prev = before_this.item->prev;
		item->next = before_this.item;
		before_this.item->prev = item;
		item->prev->next = item;
		return iterator(*this, item, 0);
	}
};