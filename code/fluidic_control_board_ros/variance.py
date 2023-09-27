import statistics

# Load length values from the text document
filename = 'length_values.txt'
with open(filename, 'r') as f:
    length_values = [float(line.strip()) for line in f]

# Calculate statistics
average = statistics.mean(length_values)
median = statistics.median(length_values)
mode = statistics.mode(length_values)
standard_deviation = statistics.stdev(length_values)
population_variance = statistics.pvariance(length_values)
sample_variance = statistics.variance(length_values)
lowest_value = min(length_values)
highest_value = max(length_values)

# Print statistics
print("Lowest Value:", lowest_value)
print("Highest Value:", highest_value)
print("Average (Mean):", average)
print("Median:", median)
print("Mode:", mode)
print("Standard Deviation:", standard_deviation)
print("Population Variance:", population_variance)
print("Sample Variance:", sample_variance)