import pandas as pd

def print_full(x):
    pd.set_option('display.max_rows', None)
    pd.set_option('display.max_columns', None)
    pd.set_option('display.width', 2000)
    pd.set_option('display.float_format', '{:20,.2f}'.format)
    pd.set_option('display.max_colwidth', None)
    print(x)
    pd.reset_option('display.max_rows')
    pd.reset_option('display.max_columns')
    pd.reset_option('display.width')
    pd.reset_option('display.float_format')
    pd.reset_option('display.max_colwidth')


data = pd.read_csv('build/benchmark.csv')
pd.set_option('display.max_colwidth', None)

data.columns = ['Date', 'Time', 'Solved', 'Name', 'N vertices', 'Total queries', 'Free queries', 'Duration']
for column in ['N vertices', 'Total queries', 'Free queries', 'Duration']:
    data[column] = data[column].astype(float)
data_groups = data.groupby(by=data['Name'])

rows_list = []
for group in data_groups:
    mean_vertices = group[1]['N vertices'].mean(axis=0)
    mean_total_queries = group[1]['Total queries'].mean(axis=0)
    mean_free_queries = group[1]['Free queries'].mean(axis=0)
    mean_duration = group[1]['Duration'].mean(axis=0)
    rows_list.append([group[0], mean_vertices, mean_total_queries, mean_free_queries, mean_duration])
data_mean = pd.DataFrame(rows_list, columns=data.columns[3:])
print_full(data_mean)
