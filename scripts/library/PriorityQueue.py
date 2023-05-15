#!/usr/bin/env python
import heapq

class PriorityQueue:
    def __init__(self):
        self._queue = []
        self._index = 0

    def is_empty(self):
        return len(self._queue) == 0

    def push(self, item, priority):
        heapq.heappush(self._queue, (priority, self._index, item))
        self._index += 1

    def pop(self):
        if not self.is_empty():
            return heapq.heappop(self._queue)[-1]
        raise IndexError("Priority queue is empty")

    def peek(self):
        if not self.is_empty():
            return self._queue[0][-1]
        raise IndexError("Priority queue is empty")

    def get_length(self):
        return len(self._queue)
