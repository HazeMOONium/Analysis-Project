Problem modeling
Problem in form of a story:
Mr Mekhaimer runs a medium-sized warehouse in a small town in Beni Suef. For years, everything was manual: counting stock, calling suppliers, and assigning workers to orders based on guesswork or the nearest deadline. Now that the warehouse has grown, late deliveries and wasted travel time have started to hurt customer trust and profit. He wants a system that can build a next-day plan that is feasible, fair to workers, and cost-aware.

The warehouse itself is a real place with a travel map. The dispatch door is marked "D", shelves are numbered nodes, and there may be a supplier node "V" for restocking. Moving between nodes takes time, and the best path between any two shelves is not always direct. The system needs these travel times to plan a route that starts and ends at dispatch.

Every item has a shelf, a weight, and a type. Fragile items force slower handling, expensive items may require extra checks, and perishable items have a maximum wait time before pick. Stock is limited, so if demand exceeds what is on the shelf, a restock delivery must be scheduled. Restock deliveries are limited by supplier reserve, preparation time, delivery time, and a weight cap per trip, and they can be delayed if the risk is high.

Orders arrive with release times and deadlines. Each order has lines (item and quantity), a priority (VIP, URGENT, NORMAL), and a packing or verification time after picking. Mr Brown wants urgent and VIP orders handled first, but deadlines must still be met and late dispatch is unacceptable. Orders that are out of stock should wait for restock rather than being discarded.

Workers (pickers) operate in shifts with limited capacity per trip. A picker cannot carry more than a maximum weight, and cannot stay past shift end except for a limited overtime buffer. A batch of orders assigned to a picker must start after the picker is available and after all order release times. The batch must finish before the overtime limit and dispatch before each order deadline.

To keep planning realistic, Mr Mekhaimer assumes that all orders for the next day are known in advance, travel times are fixed, and time is measured in minutes. Initial stock is available at time 0, and any restock only increases stock when it arrives. Perishable items must be picked within their max wait time counted from the last restock arrival (or time 0 for initial stock). Each order is placed in exactly one batch and each batch is handled by one picker.

If the plan satisfies all constraints, it outputs the list of restock requests and pick batches, with times, routes, and stock changes. If not, it must report infeasibility with a clear reason such as insufficient reserve stock, a missed deadline, a stock violation, or an overtime limit breach.
