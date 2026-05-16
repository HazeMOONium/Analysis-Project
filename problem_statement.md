# Problem Formulation (Based on ai_solution.cpp)

## Objective
Construct a feasible next-day warehouse operations plan that assigns customer orders to pick batches and pickers, schedules any needed restock deliveries, and routes each batch through shelves to meet deadlines and shift constraints while respecting stock, capacity, and handling-time effects. Minimzing total completion time, total picker overtime, total travel time, the number of restock requests, and perishable risk is a plus.

## Inputs
1. Warehouse graph
   - Nodes include dispatch "D", supplier "V" (optional), and shelves.
   - Undirected weighted edges represent travel time between nodes.
   - All-pairs shortest travel times are used for routing.

2. Items
   - `id`, `shelf` node
   - `stock` (initial units)
   - `weight` (kg per unit)
   - `itype` in {NORMAL, FRAGILE, EXPENSIVE, PERISHABLE}
   - `handle_time` (extra minutes per unit when picking)
   - `max_wait` (perishable only; maximum minutes allowed on shelf before pick)

3. Orders
   - `id`
   - `lines`: list of (item_id, quantity)
   - `release` (earliest start time)
   - `deadline` (latest dispatch time)
   - `priority` in {VIP, URGENT, NORMAL}
   - `pack_time` (minutes of packing/verification after picking)

4. Pickers (workers)
   - `id`
   - `shift_start`, `shift_end`
   - `capacity` (max kg per batch)
   - `overtime_limit` (allowed minutes after shift end)

5. Restock info (optional per item)
   - `item_id`
   - `reserve` (supplier units available)
   - `prep_time`, `delivery_time`
   - `max_weight` per delivery (kg)
   - `delay_risk`, `delay_buffer` (extra time if risk is high)

## Decision Variables (Plan Output)
1. Restock requests
   - `item_id`, `qty`, `creation_time`, `arrival_time`, `shelf`
2. Pick batches
   - `picker_id`
   - `order_ids` assigned to the batch
   - `start_time`, `finish_time`, `dispatch_time`
   - `shelf_sequence` route starting/ending at dispatch "D"
   - `total_weight`

## Constraints
1. Stock feasibility
   - Running stock for each item must never drop below zero.
   - Restock arrivals increase stock only at or after their arrival time.

2. Restock feasibility
   - For any item with demand exceeding initial stock, restock deliveries must be scheduled.
   - Total restocked units cannot exceed supplier reserve.
   - Each restock delivery cannot exceed `max_weight`.

3. Picker capacity
   - Sum of weights of all order lines in a batch must not exceed picker capacity.

4. Release and shift feasibility
   - A batch can only start at or after the assigned picker's availability and after each order's release time.
   - Picker finish time must not exceed `shift_end + overtime_limit`.

5. Deadline feasibility
   - Each order's dispatch time must be no later than its deadline.

6. Perishable handling
   - For perishable items, the elapsed time between last restock arrival and pick time must not exceed `max_wait`.

7. Routing feasibility
   - Route is a nearest-neighbor tour over the shelves needed by the batch, starting and ending at "D".
   - Batch finish time includes travel time, per-shelf service time (units * (1 + handle_time)), and packing time.

## Feasibility Output
- If all constraints are satisfied, output the full plan of restocks and batches.
- If any constraint fails, report INFEASIBLE with a specific reason (e.g., insufficient reserve stock, missed deadline, stock violation, overtime limit).

## Assumptions
1. All times are in minutes; release and deadline are fixed inputs and not modified.
2. The warehouse graph is connected, travel times are deterministic, and all-pairs shortest paths are valid.
3. Orders are known at planning time and are not split across multiple days.
4. Each order is assigned to exactly one batch; each batch is executed by one picker.
5. Picker capacity is the sum of item weights in the batch.
6. Packing/verification time is applied after picking is complete (at dispatch).
7. Initial stock is available at time 0; restock arrivals increase stock only at or after their arrival time.
8. For perishable items, max_wait is measured from the last restock arrival (or time 0 for initial stock) to pick time.
9. Routing uses a nearest-neighbor heuristic; optimal routing is not required.
